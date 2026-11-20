//! Bounded incompressible source-panel potential flow for closed meshes.
//!
//! This is a deliberate, small-scale engineering model: each mesh triangle is
//! one constant-strength source panel, the no-penetration system is solved by
//! dense Gaussian elimination, and surface pressure is recovered from the
//! tangential perturbation velocity. It is not a general panel-method package,
//! a boundary-layer solver or a lifting-surface solver.
//!
//! Assumptions:
//! - one closed, outward-wound triangle mesh with no overlapping/internal
//!   surfaces, no separate wake and no circulation;
//! - isolated body in steady, uniform, incompressible flow;
//! - Mach ≤ 0.2 and at most [`MAX_POTENTIAL_PANELS`] triangles;
//! - no rotation, wind gradient, viscosity, separation, base pressure or
//!   boundary layer. The pressure load is inviscid and has no Kutta condition.
//!
//! The cost is `O(N³)` for the solve and `O(N²)` scratch memory. Exceeding the
//! panel budget, supplying an open/disconnected mesh or a singular panel system
//! is an explicit error, never a partial result. The panel system is rebuilt
//! for each evaluation; callers must budget this opt-in model on the step path.

use super::pressure::{PressureError, PressureFlow, PressureWrench, closed_mesh_pressure_patches};
use super::wing::{FiniteWing, WingSurface};
use crate::shape::TriangleMesh;
use crate::transform::Transform;
use kapitsa_math::{Quat, Vec3};
use std::f64::consts::PI;

/// Maximum mesh triangles accepted by the source-panel model.
pub const MAX_POTENTIAL_PANELS: usize = 256;

#[derive(Clone, Copy, Debug)]
struct Panel {
    center: Vec3,
    normal: Vec3,
    area: f64,
}

/// Evaluates inviscid incompressible potential flow over a closed mesh.
///
/// The returned wrench is in world coordinates and its moment is about
/// `flow.center_of_mass`, matching the other pressure evaluators. The caller
/// must establish the closed/outward-wound, isolated, non-overlapping geometry
/// assumptions and the Mach ≤ 0.2 domain. Empty or non-closed meshes, meshes
/// above the panel budget, rotation, invalid flow and singular systems are
/// explicit errors; a representable result is never clamped.
pub fn evaluate_closed_mesh(
    mesh: &TriangleMesh,
    flow: PressureFlow,
) -> Result<PressureWrench, crate::Error> {
    if !flow.center_of_mass.is_finite()
        || !flow.linear_velocity.is_finite()
        || !flow.angular_velocity.is_finite()
    {
        return Err(crate::Error::from(PressureError::InvalidMotion));
    }
    if !flow.sound_speed.is_finite() || flow.sound_speed <= 0.0 {
        return Err(crate::Error::from(PressureError::InvalidSoundSpeed));
    }
    if mesh.triangles().is_empty() {
        return Err(crate::Error::from(PressureError::InvalidPatch));
    }
    if mesh.triangles().len() > MAX_POTENTIAL_PANELS {
        return Err(crate::Error::from(PressureError::WorkBudgetExceeded));
    }
    // Exact closed-mesh validation also rejects repeated edges, reversed
    // winding and disconnected components.
    closed_mesh_pressure_patches(mesh)?;

    let relative = flow
        .linear_velocity
        .checked_sub(flow.atmosphere.wind_velocity())
        .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
    let speed = relative
        .checked_length()
        .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
    let angular_speed = flow
        .angular_velocity
        .checked_length()
        .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
    // A uniform source-panel solve cannot represent rotation. Absorb only
    // roundoff-scale angular velocity left by a force-free integration step.
    if angular_speed > 64.0 * f64::EPSILON * speed.max(1.0) {
        return Err(crate::Error::from(PressureError::UnsupportedGeometry));
    }
    let zero = PressureWrench {
        force: Vec3::ZERO,
        moment: Vec3::ZERO,
    };
    if speed == 0.0 || flow.atmosphere.density() == 0.0 {
        return Ok(zero);
    }
    if speed / flow.sound_speed > 0.2 {
        return Err(crate::Error::from(PressureError::UnsupportedMach));
    }

    let panels = panels(mesh, flow)?;
    let count = panels.len();
    let matrix_len = count
        .checked_mul(count)
        .ok_or_else(|| crate::Error::from(PressureError::WorkBudgetExceeded))?;
    let mut matrix = Vec::new();
    matrix
        .try_reserve_exact(matrix_len)
        .map_err(|_| crate::Error::from(PressureError::WorkBudgetExceeded))?;
    matrix.resize(matrix_len, 0.0);
    let mut rhs = Vec::new();
    rhs.try_reserve_exact(count)
        .map_err(|_| crate::Error::from(PressureError::WorkBudgetExceeded))?;
    rhs.resize(count, 0.0);

    let four_pi = 4.0 * PI;
    for i in 0..count {
        let pi = panels[i];
        rhs[i] = -relative
            .checked_dot(pi.normal)
            .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
        for j in 0..count {
            if i == j {
                matrix[i * count + j] = -0.5;
                continue;
            }
            let pj = panels[j];
            let offset = pi
                .center
                .checked_sub(pj.center)
                .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
            let distance = offset
                .checked_length()
                .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
            if distance == 0.0 {
                return Err(crate::Error::from(PressureError::InvalidPatch));
            }
            let normal_velocity = pi
                .normal
                .checked_dot(offset)
                .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?
                / (four_pi * distance * distance * distance);
            matrix[i * count + j] = pj.area * normal_velocity;
        }
    }
    let strengths = solve_dense(matrix, rhs, count)?;

    let world_com = flow.transform.transform_point(flow.center_of_mass);
    let dynamic_pressure = 0.5 * flow.atmosphere.density() * speed * speed;
    let mut total = zero;
    for i in 0..count {
        let panel = panels[i];
        let mut induced = Vec3::ZERO;
        for j in 0..count {
            if i == j {
                continue;
            }
            let offset = panel
                .center
                .checked_sub(panels[j].center)
                .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
            let distance = offset
                .checked_length()
                .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
            if distance == 0.0 {
                return Err(crate::Error::from(PressureError::InvalidPatch));
            }
            let scale = strengths[j] * panels[j].area / (four_pi * distance * distance * distance);
            let contribution = offset
                .checked_mul_scalar(scale)
                .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
            induced = induced
                .checked_add(contribution)
                .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
        }
        let velocity = relative
            .checked_add(induced)
            .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
        let normal_speed = velocity
            .checked_dot(panel.normal)
            .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
        let tangential = velocity
            .checked_sub(
                panel
                    .normal
                    .checked_mul_scalar(normal_speed)
                    .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?,
            )
            .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
        let tangential_speed = tangential
            .checked_length()
            .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
        let cp = 1.0 - (tangential_speed / speed).powi(2);
        if !cp.is_finite() {
            return Err(crate::Error::from(PressureError::ArithmeticOverflow));
        }
        let area_pressure = dynamic_pressure * cp * panel.area;
        if !area_pressure.is_finite() {
            return Err(crate::Error::from(PressureError::ArithmeticOverflow));
        }
        let force = panel
            .normal
            .checked_mul_scalar(-area_pressure)
            .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
        let arm = panel
            .center
            .checked_sub(world_com)
            .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
        let moment = arm
            .checked_cross(force)
            .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
        total.force = total
            .force
            .checked_add(force)
            .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
        total.moment = total
            .moment
            .checked_add(moment)
            .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
    }
    if !total.force.is_finite() || !total.moment.is_finite() {
        return Err(crate::Error::from(PressureError::ArithmeticOverflow));
    }
    Ok(total)
}

/// Evaluates a Kutta-condition lifting-line for a closed thin wing mesh.
///
/// The mesh must be a single closed, outward-wound wing-like body in the
/// body-local axes: chord along `+X`, span along `+Y`, lift along `+Z`. Span,
/// chord and projected planform area are measured from the mesh; planform
/// aspect ratio is `span² / area`, and the quarter-chord point becomes the load
/// application point. The existing finite-wing lifting-line is evaluated there,
/// so its trailing-vortex induced-downwash wake and Kutta condition are used.
/// The caller must establish a thin uncambered wing (`thickness <= 0.15 span`),
/// `span >= chord`, attached small incidence, and Mach <= 0.3.
///
/// This is an equivalent-wing bridge, not a general three-dimensional Kutta
/// panel method: camber, sweep, taper, tip relief, separated wakes, thickness
/// and control surfaces are not resolved. Non-closed or open geometry,
/// non-wing-like bounds and Mach/attached-flow limits return explicit errors.
pub fn evaluate_kutta_lifting_mesh(
    mesh: &TriangleMesh,
    flow: PressureFlow,
) -> Result<PressureWrench, crate::Error> {
    if mesh.triangles().is_empty() {
        return Err(crate::Error::from(PressureError::InvalidPatch));
    }
    closed_mesh_pressure_patches(mesh)?;
    let mut lower = Vec3::splat(f64::INFINITY);
    let mut upper = Vec3::splat(f64::NEG_INFINITY);
    let mut projected_area = 0.0;
    for face in mesh.triangles() {
        let [a, b, c] = face.map(|index| mesh.vertices()[index]);
        lower = Vec3::new(lower.x.min(a.x), lower.y.min(a.y), lower.z.min(a.z));
        lower = Vec3::new(lower.x.min(b.x), lower.y.min(b.y), lower.z.min(b.z));
        lower = Vec3::new(lower.x.min(c.x), lower.y.min(c.y), lower.z.min(c.z));
        upper = Vec3::new(upper.x.max(a.x), upper.y.max(a.y), upper.z.max(a.z));
        upper = Vec3::new(upper.x.max(b.x), upper.y.max(b.y), upper.z.max(b.z));
        upper = Vec3::new(upper.x.max(c.x), upper.y.max(c.y), upper.z.max(c.z));
        let cross = b
            .checked_sub(a)
            .and_then(|u| c.checked_sub(a).and_then(|v| u.checked_cross(v)))
            .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
        projected_area += cross.z.abs() * 0.5;
    }
    let span = upper.y - lower.y;
    let chord = upper.x - lower.x;
    let thickness = upper.z - lower.z;
    let area = projected_area * 0.5;
    if !span.is_finite()
        || !chord.is_finite()
        || !thickness.is_finite()
        || !area.is_finite()
        || span <= 0.0
        || chord <= 0.0
        || area <= 0.0
        || span < chord
        || thickness > 0.15 * span
    {
        return Err(crate::Error::from(PressureError::UnsupportedGeometry));
    }
    let aspect_ratio = span * span / area;
    let model = FiniteWing::new(aspect_ratio, 0.8, 0.0, 0.0, 0.35)?;
    let quarter_chord = Vec3::new(
        lower.x + 0.25 * chord,
        0.5 * (lower.y + upper.y),
        0.5 * (lower.z + upper.z),
    );
    let surface = WingSurface::new(model, Transform::new(quarter_chord, Quat::IDENTITY)?, area)?;
    surface.evaluate(flow)
}

fn panels(mesh: &TriangleMesh, flow: PressureFlow) -> Result<Vec<Panel>, crate::Error> {
    let mut panels = Vec::new();
    panels
        .try_reserve_exact(mesh.triangles().len())
        .map_err(|_| crate::Error::from(PressureError::WorkBudgetExceeded))?;
    for face in mesh.triangles() {
        let [a, b, c] = face.map(|index| flow.transform.transform_point(mesh.vertices()[index]));
        let cross = b
            .checked_sub(a)
            .and_then(|u| c.checked_sub(a).and_then(|v| u.checked_cross(v)))
            .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
        let length = cross
            .checked_length()
            .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
        if length == 0.0 {
            return Err(crate::Error::from(PressureError::InvalidPatch));
        }
        panels.push(Panel {
            center: a / 3.0 + b / 3.0 + c / 3.0,
            normal: cross / length,
            area: length * 0.5,
        });
    }
    Ok(panels)
}

pub(crate) fn solve_dense(
    mut matrix: Vec<f64>,
    mut rhs: Vec<f64>,
    count: usize,
) -> Result<Vec<f64>, crate::Error> {
    for column in 0..count {
        let mut pivot = column;
        for row in (column + 1)..count {
            if matrix[row * count + column].abs() > matrix[pivot * count + column].abs() {
                pivot = row;
            }
        }
        if !matrix[pivot * count + column].is_finite()
            || matrix[pivot * count + column].abs() < 1e-14
        {
            return Err(crate::Error::from(PressureError::ArithmeticOverflow));
        }
        if pivot != column {
            for index in column..count {
                matrix.swap(column * count + index, pivot * count + index);
            }
            rhs.swap(column, pivot);
        }
        let diagonal = matrix[column * count + column];
        for row in (column + 1)..count {
            let factor = matrix[row * count + column] / diagonal;
            if factor == 0.0 {
                continue;
            }
            for index in column..count {
                matrix[row * count + index] -= factor * matrix[column * count + index];
            }
            rhs[row] -= factor * rhs[column];
        }
    }
    for row in (0..count).rev() {
        let mut value = rhs[row];
        for column in (row + 1)..count {
            value -= matrix[row * count + column] * rhs[column];
        }
        let diagonal = matrix[row * count + row];
        if !diagonal.is_finite() || diagonal.abs() < 1e-14 {
            return Err(crate::Error::from(PressureError::ArithmeticOverflow));
        }
        rhs[row] = value / diagonal;
    }
    if rhs.iter().any(|value| !value.is_finite()) {
        return Err(crate::Error::from(PressureError::ArithmeticOverflow));
    }
    Ok(rhs)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aerodynamics::AtmosphericState;
    use crate::aerodynamics::colliders::{ColliderTessellation, collider_mesh};
    use crate::collider::Collider;
    use crate::shape::{Shape, Sphere};
    use crate::transform::Transform;

    fn sphere_mesh() -> Result<TriangleMesh, crate::Error> {
        let collider = Collider::new(Shape::Sphere(Sphere::new(1.0)?), Transform::identity());
        collider_mesh([&collider], ColliderTessellation::new(16, 9, 1000)?)
    }

    fn flow(speed: f64) -> Result<PressureFlow, crate::Error> {
        Ok(PressureFlow {
            atmosphere: AtmosphericState::new(1.0, Vec3::ZERO)?,
            sound_speed: 340.0,
            transform: Transform::identity(),
            center_of_mass: Vec3::ZERO,
            linear_velocity: Vec3::X * speed,
            angular_velocity: Vec3::ZERO,
        })
    }

    #[test]
    fn closed_sphere_potential_pressure_has_zero_net_force() -> Result<(), crate::Error> {
        let mesh = sphere_mesh()?;
        let wrench = evaluate_closed_mesh(&mesh, flow(20.0)?)?;
        println!("force={:?} moment={:?}", wrench.force, wrench.moment);
        assert!(wrench.force.length() < 1e-6, "{:?}", wrench.force);
        assert!(wrench.moment.length() < 1e-6, "{:?}", wrench.moment);
        Ok(())
    }

    #[test]
    fn kutta_lifting_mesh_requires_a_thin_closed_wing() -> Result<(), crate::Error> {
        use crate::shape::Cuboid;
        let wing = collider_mesh(
            [&Collider::new(
                Shape::Cuboid(Cuboid::new(1.0, 4.0, 0.1)?),
                Transform::identity(),
            )],
            ColliderTessellation::new(16, 9, 1000)?,
        )?;
        let mut input = flow(20.0)?;
        input.linear_velocity = Vec3::new(20.0, 0.0, -2.0);
        let wrench = evaluate_kutta_lifting_mesh(&wing, input)?;
        assert!(wrench.force.length() > 0.0, "{:?}", wrench.force);
        assert!(wrench.force.z > 0.0, "{:?}", wrench.force);

        let cube = collider_mesh(
            [&Collider::new(
                Shape::Cuboid(Cuboid::new(1.0, 1.0, 1.0)?),
                Transform::identity(),
            )],
            ColliderTessellation::new(16, 9, 1000)?,
        )?;
        assert_eq!(
            evaluate_kutta_lifting_mesh(&cube, input),
            Err(crate::Error::from(PressureError::UnsupportedGeometry))
        );
        let open = TriangleMesh::new(vec![Vec3::ZERO, Vec3::Y, Vec3::Z], vec![[0, 1, 2]])?;
        assert_eq!(
            evaluate_kutta_lifting_mesh(&open, input),
            Err(crate::Error::from(PressureError::InvalidPatch))
        );
        assert_eq!(
            evaluate_kutta_lifting_mesh(
                &wing,
                PressureFlow {
                    sound_speed: 50.0,
                    ..input
                }
            ),
            Err(crate::Error::from(
                crate::aerodynamics::wing::WingError::UnsupportedFlow
            ))
        );
        Ok(())
    }

    #[test]
    fn potential_panel_model_checks_geometry_mach_and_motion() -> Result<(), crate::Error> {
        let mesh = sphere_mesh()?;
        assert!(evaluate_closed_mesh(&mesh, flow(20.0)?)?.force.length() < 1e-10);
        assert_eq!(
            evaluate_closed_mesh(
                &mesh,
                PressureFlow {
                    sound_speed: 50.0,
                    ..flow(20.0)?
                }
            ),
            Err(crate::Error::from(PressureError::UnsupportedMach))
        );
        assert_eq!(
            evaluate_closed_mesh(
                &mesh,
                PressureFlow {
                    angular_velocity: Vec3::Z,
                    ..flow(20.0)?
                }
            ),
            Err(crate::Error::from(PressureError::UnsupportedGeometry))
        );
        let open = TriangleMesh::new(vec![Vec3::ZERO, Vec3::Y, Vec3::Z], vec![[0, 1, 2]])?;
        assert_eq!(
            evaluate_closed_mesh(&open, flow(20.0)?),
            Err(crate::Error::from(PressureError::InvalidPatch))
        );
        let too_many = TriangleMesh::new(
            vec![Vec3::ZERO, Vec3::X, Vec3::Y],
            (0..=MAX_POTENTIAL_PANELS)
                .map(|_| [0, 1, 2])
                .collect::<Vec<_>>(),
        )?;
        assert_eq!(
            evaluate_closed_mesh(&too_many, flow(20.0)?),
            Err(crate::Error::from(PressureError::WorkBudgetExceeded))
        );
        Ok(())
    }
}
