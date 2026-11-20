//! Bounded vortex-lattice method for a closed thin wing mesh.
//!
//! The mesh is sampled into a spanwise/chordwise lifting lattice. Each panel
//! carries one horseshoe vortex with a bound quarter-chord leg and two flat
//! trailing legs. Circulation is solved from the no-penetration condition at
//! the three-quarter-chord control points, so the trailing wake imposes the
//! Kutta condition. Local leading and trailing edges account for sweep and
//! taper; mean sampled height approximates camber; finite span endpoints give
//! tip relief. This is still an engineering model: it assumes a closed,
//! isolated, thin, uncambered-to-mildly-cambered attached wing with no
//! thickness, separation, wake roll-up, viscosity, compressibility above 0.3,
//! control surfaces or interference.

use super::potential::solve_dense;
use super::pressure::{PressureError, PressureFlow, PressureWrench, closed_mesh_pressure_patches};
use crate::shape::TriangleMesh;
use kapitsa_math::Vec3;
use std::f64::consts::PI;

/// Fixed lattice resolution. The dense solve is bounded to 18 panels.
pub const VLM_CHORD_PANELS: usize = 3;
/// Fixed spanwise lattice resolution.
pub const VLM_SPAN_PANELS: usize = 6;
const MAX_VLM_MACH: f64 = 0.3;

#[derive(Clone, Copy, Debug)]
struct LatticePanel {
    quarter_a: Vec3,
    quarter_b: Vec3,
    control: Vec3,
    normal: Vec3,
}

/// Evaluates a bounded vortex-lattice wing model on a closed thin wing mesh.
///
/// Body-local `+X` is chord, `+Y` span and `+Z` lift. The mesh must be closed,
/// outward wound, `span >= chord`, `thickness <= 0.15 span`, Mach <= 0.3, and
/// the incidence must remain within the attached-flow range used by the
/// lattice. The result is a world-space wrench about `flow.center_of_mass`.
/// Open/non-wing geometry, rotation, singular wakes, allocation failure and
/// nonfinite loads are explicit errors.
pub fn evaluate_closed_wing(
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
    if flow.angular_velocity != Vec3::ZERO {
        return Err(crate::Error::from(PressureError::UnsupportedGeometry));
    }
    if mesh.triangles().is_empty() {
        return Err(crate::Error::from(PressureError::InvalidPatch));
    }
    closed_mesh_pressure_patches(mesh)?;
    let relative = flow
        .linear_velocity
        .checked_sub(flow.atmosphere.wind_velocity())
        .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
    let speed = relative
        .checked_length()
        .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
    let zero = PressureWrench {
        force: Vec3::ZERO,
        moment: Vec3::ZERO,
    };
    if speed == 0.0 || flow.atmosphere.density() == 0.0 {
        return Ok(zero);
    }
    if speed / flow.sound_speed > MAX_VLM_MACH {
        return Err(crate::Error::from(PressureError::UnsupportedMach));
    }

    let lattice = lattice(mesh)?;
    let count = lattice.len();
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
    let free = relative;
    for i in 0..count {
        let panel = lattice[i];
        rhs[i] = -free
            .checked_dot(panel.normal)
            .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
        for (j, source) in lattice.iter().enumerate() {
            let induced = horseshoe(*source, panel.control)?;
            matrix[i * count + j] = -induced
                .checked_dot(panel.normal)
                .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
        }
    }
    let strengths = solve_dense(matrix, rhs, count)?;
    let world_com = flow.transform.transform_point(flow.center_of_mass);
    let mut total = zero;
    for i in 0..count {
        let panel = lattice[i];
        let center = panel
            .quarter_a
            .checked_add(panel.quarter_b)
            .and_then(|v| v.checked_mul_scalar(0.5))
            .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
        let mut local = free;
        for (j, source) in lattice.iter().enumerate() {
            if i == j {
                continue;
            }
            let induced = horseshoe(*source, center)?;
            local = local
                .checked_add(induced)
                .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
        }
        let span = panel
            .quarter_b
            .checked_sub(panel.quarter_a)
            .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
        let force = local
            .checked_cross(span)
            .and_then(|v| v.checked_mul_scalar(strengths[i] * flow.atmosphere.density()))
            .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
        if !force.is_finite() {
            return Err(crate::Error::from(PressureError::ArithmeticOverflow));
        }
        let arm = center
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

fn lattice(mesh: &TriangleMesh) -> Result<Vec<LatticePanel>, crate::Error> {
    let mut lower = Vec3::splat(f64::INFINITY);
    let mut upper = Vec3::splat(f64::NEG_INFINITY);
    let mut projected_area = 0.0;
    for face in mesh.triangles() {
        let [a, b, c] = face.map(|index| mesh.vertices()[index]);
        lower = min_point(lower, a);
        lower = min_point(lower, b);
        lower = min_point(lower, c);
        upper = max_point(upper, a);
        upper = max_point(upper, b);
        upper = max_point(upper, c);
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
    let stations = span_stations(mesh)?;
    let mut panels = Vec::new();
    panels
        .try_reserve_exact(VLM_CHORD_PANELS * VLM_SPAN_PANELS)
        .map_err(|_| crate::Error::from(PressureError::WorkBudgetExceeded))?;
    for span_index in 0..VLM_SPAN_PANELS {
        let y0 = lower.y + span * span_index as f64 / VLM_SPAN_PANELS as f64;
        let y1 = lower.y + span * (span_index + 1) as f64 / VLM_SPAN_PANELS as f64;
        let (leading, trailing) = edges_at(&stations, y0, y1)?;
        let chord_span = trailing - leading;
        if !chord_span.is_finite() || chord_span <= 0.0 {
            return Err(crate::Error::from(PressureError::UnsupportedGeometry));
        }
        for chord_index in 0..VLM_CHORD_PANELS {
            let f0 = chord_index as f64 / VLM_CHORD_PANELS as f64;
            let f1 = (chord_index + 1) as f64 / VLM_CHORD_PANELS as f64;
            let x00 = leading + chord_span * f0;
            let x01 = leading + chord_span * f1;
            let z00 = camber_at(mesh, x00, y0);
            let z01 = camber_at(mesh, x01, y0);
            let z10 = camber_at(mesh, x00, y1);
            let z11 = camber_at(mesh, x01, y1);
            let p00 = Vec3::new(x00, y0, z00);
            let p10 = Vec3::new(x01, y0, z01);
            let p01 = Vec3::new(x00, y1, z10);
            let panel_chord = x01 - x00;
            let quarter_a = Vec3::new(x00 + 0.25 * panel_chord, y0, z00);
            let quarter_b = Vec3::new(x00 + 0.25 * panel_chord, y1, z10);
            let control = Vec3::new(
                x00 + 0.75 * panel_chord,
                (y0 + y1) * 0.5,
                (z00 + z01 + z10 + z11) * 0.25,
            );
            let normal = p10
                .checked_sub(p00)
                .and_then(|u| p01.checked_sub(p00).and_then(|v| u.checked_cross(v)))
                .and_then(Vec3::try_normalize)
                .ok_or_else(|| crate::Error::from(PressureError::InvalidPatch))?;
            panels.push(LatticePanel {
                quarter_a,
                quarter_b,
                control,
                normal,
            });
        }
    }
    Ok(panels)
}

#[derive(Clone, Copy, Debug)]
struct SpanStation {
    y: f64,
    leading: f64,
    trailing: f64,
}

fn span_stations(mesh: &TriangleMesh) -> Result<Vec<SpanStation>, crate::Error> {
    let mut stations: Vec<SpanStation> = Vec::new();
    for vertex in mesh.vertices() {
        if let Some(station) = stations.iter_mut().find(|station| station.y == vertex.y) {
            station.leading = station.leading.min(vertex.x);
            station.trailing = station.trailing.max(vertex.x);
        } else {
            stations.push(SpanStation {
                y: vertex.y,
                leading: vertex.x,
                trailing: vertex.x,
            });
        }
    }
    stations.sort_by(|a, b| a.y.total_cmp(&b.y));
    if stations.len() < 2 {
        return Err(crate::Error::from(PressureError::UnsupportedGeometry));
    }
    Ok(stations)
}

fn station_at(stations: &[SpanStation], y: f64) -> Result<(f64, f64), crate::Error> {
    if y <= stations[0].y {
        return Ok((stations[0].leading, stations[0].trailing));
    }
    for pair in stations.windows(2) {
        let [a, b] = [pair[0], pair[1]];
        if y <= b.y {
            let t = (y - a.y) / (b.y - a.y);
            return Ok((
                a.leading + t * (b.leading - a.leading),
                a.trailing + t * (b.trailing - a.trailing),
            ));
        }
    }
    let last = stations[stations.len() - 1];
    Ok((last.leading, last.trailing))
}

fn edges_at(stations: &[SpanStation], y0: f64, y1: f64) -> Result<(f64, f64), crate::Error> {
    let (leading0, trailing0) = station_at(stations, y0)?;
    let (leading1, trailing1) = station_at(stations, y1)?;
    let leading = leading0.min(leading1);
    let trailing = trailing0.max(trailing1);
    if !leading.is_finite() || !trailing.is_finite() || trailing <= leading {
        return Err(crate::Error::from(PressureError::UnsupportedGeometry));
    }
    Ok((leading, trailing))
}

fn camber_at(mesh: &TriangleMesh, x: f64, y: f64) -> f64 {
    let mut sum = 0.0;
    let mut count = 0;
    for vertex in mesh.vertices() {
        if (vertex.x - x).abs() <= 0.125 && (vertex.y - y).abs() <= 0.125 {
            sum += vertex.z;
            count += 1;
        }
    }
    if count == 0 { 0.0 } else { sum / count as f64 }
}

fn horseshoe(panel: LatticePanel, point: Vec3) -> Result<Vec3, crate::Error> {
    let far = 1000.0;
    let down = Vec3::X * far;
    let mut total = segment(panel.quarter_a, panel.quarter_b, point)?;
    let return_leg = segment(panel.quarter_a, panel.quarter_a + down, point)?
        .checked_mul_scalar(-1.0)
        .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
    total = total
        .checked_add(return_leg)
        .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
    total = total
        .checked_add(segment(panel.quarter_b, panel.quarter_b + down, point)?)
        .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
    Ok(total)
}

fn segment(a: Vec3, b: Vec3, point: Vec3) -> Result<Vec3, crate::Error> {
    let r0 = b
        .checked_sub(a)
        .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
    let r1 = point
        .checked_sub(a)
        .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
    let r2 = point
        .checked_sub(b)
        .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
    let n1 = r1
        .checked_length()
        .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
    let n2 = r2
        .checked_length()
        .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
    if n1 == 0.0 || n2 == 0.0 {
        return Ok(Vec3::ZERO);
    }
    let cross = r1
        .checked_cross(r2)
        .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
    let denominator = cross.dot(cross);
    if denominator == 0.0 {
        return Ok(Vec3::ZERO);
    }
    let difference = r1
        .checked_mul_scalar(1.0 / n1)
        .and_then(|u| {
            r2.checked_mul_scalar(1.0 / n2)
                .and_then(|v| u.checked_sub(v))
        })
        .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
    let k = r0
        .checked_dot(difference)
        .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?
        / denominator;
    cross
        .checked_mul_scalar(k / (4.0 * PI))
        .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))
}

fn min_point(a: Vec3, b: Vec3) -> Vec3 {
    Vec3::new(a.x.min(b.x), a.y.min(b.y), a.z.min(b.z))
}

fn max_point(a: Vec3, b: Vec3) -> Vec3 {
    Vec3::new(a.x.max(b.x), a.y.max(b.y), a.z.max(b.z))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aerodynamics::AtmosphericState;
    use crate::aerodynamics::colliders::{ColliderTessellation, collider_mesh};
    use crate::collider::Collider;
    use crate::shape::{Cuboid, Shape};
    use crate::transform::Transform;

    fn wing_mesh(taper_sweep_camber: bool) -> Result<TriangleMesh, crate::Error> {
        let collider = Collider::new(
            Shape::Cuboid(Cuboid::new(1.0, 4.0, 0.1)?),
            Transform::identity(),
        );
        let mesh = collider_mesh([&collider], ColliderTessellation::new(16, 9, 1000)?)?;
        if !taper_sweep_camber {
            return Ok(mesh);
        }
        let vertices = mesh
            .vertices()
            .iter()
            .map(|v| {
                Vec3::new(
                    v.x * (1.0 - 0.1 * v.y) + 0.1 * v.y,
                    v.y,
                    v.z + 0.02 * v.x * v.x,
                )
            })
            .collect();
        TriangleMesh::new(vertices, mesh.triangles().to_vec())
    }

    fn flow(speed: f64, alpha: f64) -> Result<PressureFlow, crate::Error> {
        Ok(PressureFlow {
            atmosphere: AtmosphericState::new(1.0, Vec3::ZERO)?,
            sound_speed: 340.0,
            transform: Transform::identity(),
            center_of_mass: Vec3::ZERO,
            linear_velocity: Vec3::new(speed * alpha.cos(), 0.0, -speed * alpha.sin()),
            angular_velocity: Vec3::ZERO,
        })
    }

    #[test]
    fn vortex_lattice_lifts_and_accepts_taper_sweep_camber() -> Result<(), crate::Error> {
        let wing = wing_mesh(false)?;
        let wrench = evaluate_closed_wing(&wing, flow(20.0, 0.1)?)?;
        assert!(wrench.force.z > 0.0, "{:?}", wrench.force);
        let q = 0.5 * 20.0 * 20.0;
        let cl = wrench.force.z / (q * 4.0);
        assert!(cl > 0.0 && cl < 1.0, "cl={cl}");
        let deformed = wing_mesh(true)?;
        let deformed_wrench = evaluate_closed_wing(&deformed, flow(20.0, 0.1)?)?;
        assert!(deformed_wrench.force.is_finite());
        assert!(deformed_wrench.force.z > 0.0);
        Ok(())
    }

    #[test]
    fn vortex_lattice_rejects_invalid_geometry_and_domains() -> Result<(), crate::Error> {
        let wing = wing_mesh(false)?;
        let input = flow(20.0, 0.1)?;
        let cube = collider_mesh(
            [&Collider::new(
                Shape::Cuboid(Cuboid::new(1.0, 1.0, 1.0)?),
                Transform::identity(),
            )],
            ColliderTessellation::new(16, 9, 1000)?,
        )?;
        assert_eq!(
            evaluate_closed_wing(&cube, input),
            Err(crate::Error::from(PressureError::UnsupportedGeometry))
        );
        let open = TriangleMesh::new(vec![Vec3::ZERO, Vec3::Y, Vec3::Z], vec![[0, 1, 2]])?;
        assert_eq!(
            evaluate_closed_wing(&open, input),
            Err(crate::Error::from(PressureError::InvalidPatch))
        );
        assert_eq!(
            evaluate_closed_wing(
                &wing,
                PressureFlow {
                    sound_speed: 50.0,
                    ..input
                }
            ),
            Err(crate::Error::from(PressureError::UnsupportedMach))
        );
        assert_eq!(
            evaluate_closed_wing(
                &wing,
                PressureFlow {
                    angular_velocity: Vec3::Z,
                    ..input
                }
            ),
            Err(crate::Error::from(PressureError::UnsupportedGeometry))
        );
        Ok(())
    }
}
