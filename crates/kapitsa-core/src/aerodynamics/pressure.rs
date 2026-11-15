//! Newtonian hypersonic impact pressure on caller-selected physical surface patches.
//!
//! Source: <https://en.wikipedia.org/wiki/Pressure_coefficient>, Cp = 2 sin²(theta).
//! Leeward excess pressure is zero. Mach >= 5 is an explicit implementation
//! policy, not a universal accuracy guarantee. No suction, friction, wake,
//! circulation, transonic correction or shock interaction is resolved.
//! Callers must supply exposed original-surface normals and areas. In particular,
//! voxel staircase normals are not a consistent replacement for smooth normals.
//! Overlap removal and upstream shielding are the caller's responsibility.

use super::{AtmosphericState, geometry::PreparedSurfacePatches};
use crate::transform::Transform;
use kapitsa_math::Vec3;
use std::fmt;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PressureError {
    InvalidPatch,
    InvalidMotion,
    InvalidSoundSpeed,
    InvalidCoefficient,
    InvalidBaseOrientation,
    BaseNotFullyVisible,
    UnsupportedMach,
    ArithmeticOverflow,
}
impl fmt::Display for PressureError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Newtonian pressure evaluation failed: {self:?}")
    }
}
impl std::error::Error for PressureError {}

fn same_triangle(a: &[Vec3; 3], b: &[Vec3; 3]) -> bool {
    (0..3).any(|shift| {
        (0..3).all(|i| a[i] == b[(i + shift) % 3]) || (0..3).all(|i| a[i] == b[(shift + 3 - i) % 3])
    })
}

/// Reject selected triangles that do not uniquely identify a source face.
pub(crate) fn verify_unique_base_sources(
    selected: &[[Vec3; 3]],
    mesh: &crate::shape::TriangleMesh,
    max_work: usize,
) -> Result<(), super::AerodynamicsError> {
    if selected
        .len()
        .checked_mul(mesh.triangles().len())
        .is_none_or(|work| work > max_work)
    {
        return Err(super::AerodynamicsError::Geometry(
            super::geometry::GeometryError::WorkBudgetExceeded,
        ));
    }
    for triangle in selected {
        if mesh
            .triangles()
            .iter()
            .filter(|face| same_triangle(triangle, &face.map(|index| mesh.vertices()[index])))
            .count()
            != 1
        {
            return Err(super::AerodynamicsError::Pressure(
                PressureError::BaseNotFullyVisible,
            ));
        }
    }
    Ok(())
}

/// Verifies that each selected source triangle appears unchanged exactly once
/// in an exposure result. The caller must prepare `visible` from the complete
/// current Mesh/Colliders geometry, in the direction from the base towards the
/// region being checked, and repeat this check whenever geometry or direction
/// changes. This is a geometric line-of-sight check, not a wake model.
///
/// Partial fragments, absent faces, duplicated selections and multiple matching
/// output faces are rejected. Coincident input faces may already be deduplicated
/// by geometry preparation and cannot be distinguished by this check. Exact vertex equality is deliberate: rounding or clipping may
/// conservatively reject a visible face rather than certify a blocked one.
pub fn verify_full_base_visibility(
    selected: &[[Vec3; 3]],
    visible: &PreparedSurfacePatches,
) -> Result<(), PressureError> {
    for (index, triangle) in selected.iter().enumerate() {
        if selected[..index]
            .iter()
            .any(|other| same_triangle(triangle, other))
            || visible
                .iter()
                .filter(|patch| same_triangle(triangle, &patch.vertices))
                .count()
                != 1
        {
            return Err(PressureError::BaseNotFullyVisible);
        }
    }
    Ok(())
}

/// Physical patch in a stable body-local frame, independent of center of mass.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PressurePatch {
    center: Vec3,
    normal: Vec3,
    area: f64,
}

/// Converts a consistently wound closed mesh into centroid patches.
///
/// Exactly coincident vertices are identified, including signed zero; no tolerance
/// welding is performed. Each resulting edge must occur exactly twice in opposite
/// directions and all faces must be connected. This does not prove a sphere,
/// convexity, or absence of self-intersection. It is
/// not sufficient by itself to authorize a potential-flow model in a world step.
pub fn closed_mesh_pressure_patches(
    mesh: &crate::shape::TriangleMesh,
) -> Result<Vec<PressurePatch>, PressureError> {
    use std::collections::BTreeMap;

    let faces = mesh.triangles();
    let mut vertices = BTreeMap::new();
    let mut indices = Vec::with_capacity(mesh.vertices().len());
    for vertex in mesh.vertices() {
        let key = [vertex.x, vertex.y, vertex.z].map(|v| if v == 0.0 { 0 } else { v.to_bits() });
        let next = vertices.len();
        indices.push(*vertices.entry(key).or_insert(next));
    }
    let mut edges: BTreeMap<(usize, usize), Vec<(usize, bool)>> = BTreeMap::new();
    let mut patches = Vec::with_capacity(faces.len());
    for (index, &[a, b, c]) in faces.iter().enumerate() {
        let [ia, ib, ic] = [a, b, c].map(|i| indices[i]);
        if ia == ib || ib == ic || ic == ia {
            return Err(PressureError::InvalidPatch);
        }
        for (u, v) in [(ia, ib), (ib, ic), (ic, ia)] {
            edges
                .entry((u.min(v), u.max(v)))
                .or_default()
                .push((index, u < v));
        }
        let [a, b, c] = [a, b, c].map(|i| mesh.vertices()[i]);
        let cross = b
            .checked_sub(a)
            .and_then(|u| c.checked_sub(a).and_then(|v| u.checked_cross(v)))
            .ok_or(PressureError::ArithmeticOverflow)?;
        let length = cross
            .checked_length()
            .ok_or(PressureError::ArithmeticOverflow)?;
        let center = a / 3.0 + b / 3.0 + c / 3.0;
        patches.push(PressurePatch::new(center, cross, length * 0.5)?);
    }
    let mut neighbors = vec![Vec::new(); faces.len()];
    for edge in edges.values() {
        if edge.len() != 2 || edge[0].1 == edge[1].1 {
            return Err(PressureError::InvalidPatch);
        }
        neighbors[edge[0].0].push(edge[1].0);
        neighbors[edge[1].0].push(edge[0].0);
    }
    let mut visited = vec![false; faces.len()];
    let mut stack = vec![0];
    visited[0] = true;
    while let Some(face) = stack.pop() {
        for &neighbor in &neighbors[face] {
            if !visited[neighbor] {
                visited[neighbor] = true;
                stack.push(neighbor);
            }
        }
    }
    if visited.iter().any(|seen| !seen) {
        return Err(PressureError::InvalidPatch);
    }
    Ok(patches)
}

/// Checks necessary geometric conditions for an outward-wound convex polyhedral
/// approximation of a sphere, returning its complete centroid patches. `center`
/// and `radius` are body-local; `relative_tolerance` bounds radial error relative
/// to the radius and must be finite and in [0, 1). `max_work` bounds the number
/// of face/vertex convexity checks. Exact seam matching is still required.
/// This does not establish adequate tessellation accuracy, isolation, flow
/// conditions, experimental validity, or absence of interaction with other bodies.
pub fn spherical_mesh_pressure_patches(
    mesh: &crate::shape::TriangleMesh,
    center: Vec3,
    radius: f64,
    relative_tolerance: f64,
    max_work: usize,
) -> Result<Vec<PressurePatch>, PressureError> {
    if !center.is_finite()
        || !radius.is_finite()
        || radius <= 0.0
        || !relative_tolerance.is_finite()
        || !(0.0..1.0).contains(&relative_tolerance)
    {
        return Err(PressureError::InvalidPatch);
    }
    if mesh
        .triangles()
        .len()
        .checked_mul(mesh.vertices().len())
        .is_none_or(|n| n > max_work)
    {
        return Err(PressureError::InvalidPatch);
    }
    let tolerance = radius * relative_tolerance;
    for vertex in mesh.vertices() {
        let distance = vertex
            .checked_sub(center)
            .and_then(|v| v.checked_length())
            .ok_or(PressureError::ArithmeticOverflow)?;
        if (distance - radius).abs() > tolerance {
            return Err(PressureError::InvalidPatch);
        }
    }
    let patches = closed_mesh_pressure_patches(mesh)?;
    for patch in &patches {
        let offset = patch
            .center
            .checked_sub(center)
            .ok_or(PressureError::ArithmeticOverflow)?;
        if patch
            .normal
            .checked_dot(offset)
            .ok_or(PressureError::ArithmeticOverflow)?
            <= 0.0
        {
            return Err(PressureError::InvalidPatch);
        }
        for vertex in mesh.vertices() {
            let delta = vertex
                .checked_sub(patch.center)
                .ok_or(PressureError::ArithmeticOverflow)?;
            if patch
                .normal
                .checked_dot(delta)
                .ok_or(PressureError::ArithmeticOverflow)?
                > tolerance
            {
                return Err(PressureError::InvalidPatch);
            }
        }
    }
    Ok(patches)
}

/// Evaluates potential-sphere pressure over all previously validated patches.
///
/// The caller must obtain the complete set from `spherical_mesh_pressure_patches`
/// for the current geometry and verify the isolated, non-rotating uniform-flow
/// assumptions. This function does not integrate with `PhysicsWorld`, certify
/// geometry, detect interference, or model viscous drag. An invalid patch or
/// nonfinite sum rejects the whole result.
pub fn evaluate_potential_sphere_patches(
    patches: &[PressurePatch],
    flow: PressureFlow,
) -> Result<PressureWrench, PressureError> {
    // Validate flow even for an empty set; an empty set is not a sphere.
    if patches.is_empty() {
        return Err(PressureError::InvalidPatch);
    }
    let mut total = PressureWrench {
        force: Vec3::ZERO,
        moment: Vec3::ZERO,
    };
    for patch in patches {
        let load = patch.evaluate_potential_sphere(flow)?;
        total.force = total
            .force
            .checked_add(load.force)
            .ok_or(PressureError::ArithmeticOverflow)?;
        total.moment = total
            .moment
            .checked_add(load.moment)
            .ok_or(PressureError::ArithmeticOverflow)?;
    }
    Ok(total)
}

/// Net world-space force and moment about the current center of mass.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PressureWrench {
    pub force: Vec3,
    pub moment: Vec3,
}

/// Motion and local flow at a patch. Linear velocity refers to the COM.
#[derive(Clone, Copy, Debug)]
pub struct PressureFlow {
    pub atmosphere: AtmosphericState,
    pub sound_speed: f64,
    pub transform: Transform,
    pub center_of_mass: Vec3,
    pub linear_velocity: Vec3,
    pub angular_velocity: Vec3,
}

/// Caller-supplied constant base pressure coefficient and inclusive Mach domain.
///
/// `cp` is excess pressure relative to freestream divided by dynamic pressure;
/// it must be nonpositive. This is not a wake model or a surface classifier:
/// the caller must select actual exposed base patches and provide data suitable
/// for the geometry and flow, including Reynolds number and incidence.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BasePressure {
    cp_magnitude: f64,
    min_mach: f64,
    max_mach: f64,
}

/// Explicit signed Cp and inclusive local Mach domain, not a predictive model.
/// The caller is responsible for choosing an applicable coefficient and surface.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SurfacePressure {
    cp: f64,
    min_mach: f64,
    max_mach: f64,
}

/// Maximum Mach accepted by the incompressible potential-flow approximation.
/// This conservative policy is not an experimental accuracy guarantee.
const POTENTIAL_FLOW_MAX_MACH: f64 = 0.2;
/// Squared surface-speed ratio at the equator of an ideal sphere.
const SPHERE_EQUATORIAL_SPEED_RATIO_SQUARED: f64 = 2.25;
/// Squared surface-speed ratio at the side of an ideal circular cylinder.
const CYLINDER_SIDE_SPEED_RATIO_SQUARED: f64 = 4.0;
/// Dimensionless tolerance for checking perpendicular unit vectors.
const POTENTIAL_CYLINDER_ORTHOGONALITY_TOLERANCE: f64 = 1e-10;

impl SurfacePressure {
    pub fn new(cp: f64, min_mach: f64, max_mach: f64) -> Result<Self, PressureError> {
        if !cp.is_finite()
            || !min_mach.is_finite()
            || min_mach < 0.0
            || !max_mach.is_finite()
            || max_mach < min_mach
        {
            return Err(PressureError::InvalidCoefficient);
        }
        Ok(Self {
            cp,
            min_mach,
            max_mach,
        })
    }
}

#[derive(Clone, Copy)]
enum ExplicitPressure {
    Base(BasePressure),
    Surface(SurfacePressure),
}

impl BasePressure {
    pub fn new(cp: f64, min_mach: f64, max_mach: f64) -> Result<Self, PressureError> {
        if !cp.is_finite()
            || cp > 0.0
            || !min_mach.is_finite()
            || min_mach < 0.0
            || !max_mach.is_finite()
            || max_mach < min_mach
        {
            return Err(PressureError::InvalidCoefficient);
        }
        Ok(Self {
            cp_magnitude: -cp,
            min_mach,
            max_mach,
        })
    }
}

impl PressurePatch {
    pub fn new(center: Vec3, outward_normal: Vec3, area: f64) -> Result<Self, PressureError> {
        if !center.is_finite() || !area.is_finite() || area < 0.0 {
            return Err(PressureError::InvalidPatch);
        }
        let normal = outward_normal
            .try_normalize()
            .ok_or(PressureError::InvalidPatch)?;
        Ok(Self {
            center,
            normal,
            area,
        })
    }

    pub fn center(&self) -> Vec3 {
        self.center
    }
    pub fn normal(&self) -> Vec3 {
        self.normal
    }
    pub fn area(&self) -> f64 {
        self.area
    }

    /// Potential-flow pressure on a stationary, isolated sphere in uniform flow.
    ///
    /// The caller must supply a sphere surface normal; arbitrary geometry,
    /// separation, viscosity, wake, compressibility and rotation are not modeled.
    /// Cp = 1 - (9/4) sin²(theta), with theta from the upstream stagnation point.
    /// See Barlow, Rae and Pope, *Low-Speed Wind Tunnel Testing*, 3rd ed., §4.3:
    /// <https://web.pdx.edu/~d4eb/chrome/J._B._Barlow,_W._H._Rae,_Jr,_A._Pope_Low_Speed_Wind_Tunnel_Testing.pdf>.
    /// The Mach <= 0.2 policy is an approximation, not a validation claim.
    pub fn evaluate_potential_sphere(
        &self,
        flow: PressureFlow,
    ) -> Result<PressureWrench, PressureError> {
        if flow.angular_velocity != Vec3::ZERO {
            return Err(PressureError::InvalidMotion);
        }
        // Reuse the ordinary checks even for a zero-speed or zero-area patch.
        let zero = SurfacePressure::new(0.0, 0.0, POTENTIAL_FLOW_MAX_MACH)?;
        self.evaluate_surface(flow, zero)?;
        let relative = flow
            .linear_velocity
            .checked_sub(flow.atmosphere.wind_velocity())
            .ok_or(PressureError::ArithmeticOverflow)?;
        let speed = relative
            .checked_length()
            .ok_or(PressureError::ArithmeticOverflow)?;
        if speed == 0.0 || self.area == 0.0 || flow.atmosphere.density() == 0.0 {
            return Ok(PressureWrench {
                force: Vec3::ZERO,
                moment: Vec3::ZERO,
            });
        }
        let normal = flow.transform.transform_vector(self.normal);
        let cosine = normal
            .checked_dot(relative / speed)
            .ok_or(PressureError::ArithmeticOverflow)?
            .clamp(-1.0, 1.0);
        let cp = 1.0 - SPHERE_EQUATORIAL_SPEED_RATIO_SQUARED * (1.0 - cosine * cosine);
        self.evaluate_surface(
            flow,
            SurfacePressure::new(cp, 0.0, POTENTIAL_FLOW_MAX_MACH)?,
        )
    }

    /// Inviscid incompressible potential flow around an infinitely long circular
    /// cylinder. `axis` and this side patch's normal are body-local. The caller
    /// must establish that the patch belongs to the lateral surface of an isolated
    /// cylinder; end caps, finite-length effects, separation and wake are excluded.
    /// Only crossflow and zero angular velocity are supported. Mach <= 0.2 is an
    /// approximation policy, not an experimental accuracy guarantee. Cp equals
    /// 1 - 4 sin²(theta), with theta measured from a stagnation point.
    pub fn evaluate_potential_cylinder(
        &self,
        flow: PressureFlow,
        axis: Vec3,
    ) -> Result<PressureWrench, PressureError> {
        let axis = axis.try_normalize().ok_or(PressureError::InvalidPatch)?;
        if self
            .normal
            .checked_dot(axis)
            .ok_or(PressureError::ArithmeticOverflow)?
            .abs()
            > POTENTIAL_CYLINDER_ORTHOGONALITY_TOLERANCE
        {
            return Err(PressureError::InvalidPatch);
        }
        if flow.angular_velocity != Vec3::ZERO {
            return Err(PressureError::InvalidMotion);
        }
        let zero = SurfacePressure::new(0.0, 0.0, POTENTIAL_FLOW_MAX_MACH)?;
        let zero_load = self.evaluate_surface(flow, zero)?;
        let relative = flow
            .linear_velocity
            .checked_sub(flow.atmosphere.wind_velocity())
            .ok_or(PressureError::ArithmeticOverflow)?;
        let speed = relative
            .checked_length()
            .ok_or(PressureError::ArithmeticOverflow)?;
        if speed == 0.0 || self.area == 0.0 || flow.atmosphere.density() == 0.0 {
            return Ok(zero_load);
        }
        let direction = relative / speed;
        let world_axis = flow.transform.transform_vector(axis);
        if world_axis
            .checked_dot(direction)
            .ok_or(PressureError::ArithmeticOverflow)?
            .abs()
            > POTENTIAL_CYLINDER_ORTHOGONALITY_TOLERANCE
        {
            return Err(PressureError::InvalidMotion);
        }
        let normal = flow.transform.transform_vector(self.normal);
        let cosine = normal
            .checked_dot(direction)
            .ok_or(PressureError::ArithmeticOverflow)?
            .clamp(-1.0, 1.0);
        let cp = 1.0 - CYLINDER_SIDE_SPEED_RATIO_SQUARED * (1.0 - cosine * cosine);
        self.evaluate_surface(
            flow,
            SurfacePressure::new(cp, 0.0, POTENTIAL_FLOW_MAX_MACH)?,
        )
    }

    /// Evaluates a centroid load sample; supply atmosphere sampled at this patch.
    /// COM is body-local; velocities, force and moment are world-space.
    /// No allocation, geometry preparation or body mutation occurs here.
    pub fn evaluate(&self, flow: PressureFlow) -> Result<PressureWrench, PressureError> {
        self.evaluate_inner(flow, None)
    }

    /// Applies a caller-supplied constant base Cp to this explicitly selected
    /// physical patch. Force acts outward for negative Cp. A patch whose outward
    /// normal points into its local body-relative velocity is rejected; callers
    /// must still establish that the patch is a base and is not shielded.
    pub fn evaluate_base(
        &self,
        flow: PressureFlow,
        base: BasePressure,
    ) -> Result<PressureWrench, PressureError> {
        self.evaluate_inner(flow, Some(ExplicitPressure::Base(base)))
    }

    /// Applies a supplied signed Cp without inferring exposure or wake physics.
    /// Pressure acts along the inward normal for positive Cp and outward for negative Cp.
    pub fn evaluate_surface(
        &self,
        flow: PressureFlow,
        pressure: SurfacePressure,
    ) -> Result<PressureWrench, PressureError> {
        self.evaluate_inner(flow, Some(ExplicitPressure::Surface(pressure)))
    }

    fn evaluate_inner(
        &self,
        flow: PressureFlow,
        explicit: Option<ExplicitPressure>,
    ) -> Result<PressureWrench, PressureError> {
        if !flow.center_of_mass.is_finite()
            || !flow.linear_velocity.is_finite()
            || !flow.angular_velocity.is_finite()
        {
            return Err(PressureError::InvalidMotion);
        }
        if !flow.sound_speed.is_finite() || flow.sound_speed <= 0.0 {
            return Err(PressureError::InvalidSoundSpeed);
        }
        let zero = PressureWrench {
            force: Vec3::ZERO,
            moment: Vec3::ZERO,
        };
        if self.area == 0.0 || flow.atmosphere.density() == 0.0 {
            return Ok(zero);
        }
        let overflow = PressureError::ArithmeticOverflow;
        let offset = self
            .center
            .checked_sub(flow.center_of_mass)
            .ok_or(overflow)?;
        let arm = flow.transform.transform_vector(offset);
        let normal = flow.transform.transform_vector(self.normal);
        if !arm.is_finite() || !normal.is_finite() {
            return Err(overflow);
        }
        let relative = flow
            .angular_velocity
            .checked_cross(arm)
            .and_then(|v| flow.linear_velocity.checked_add(v))
            .and_then(|v| v.checked_sub(flow.atmosphere.wind_velocity()))
            .ok_or(overflow)?;
        let speed = relative.checked_length().ok_or(overflow)?;
        if speed == 0.0 {
            return Ok(zero);
        }
        let mach = speed / flow.sound_speed;
        if !mach.is_finite() {
            return Err(overflow);
        }
        let domain = match explicit {
            Some(ExplicitPressure::Base(b)) => Some((b.min_mach, b.max_mach)),
            Some(ExplicitPressure::Surface(p)) => Some((p.min_mach, p.max_mach)),
            None => None,
        };
        if domain.map_or(mach < 5.0, |(min, max)| mach < min || mach > max) {
            return Err(PressureError::UnsupportedMach);
        }
        if let Some(explicit) = explicit {
            let cp = match explicit {
                ExplicitPressure::Base(base) => {
                    let alignment = normal.checked_dot(relative / speed).ok_or(overflow)?;
                    if alignment > 0.0 {
                        return Err(PressureError::InvalidBaseOrientation);
                    }
                    -base.cp_magnitude
                }
                ExplicitPressure::Surface(pressure) => pressure.cp,
            };
            let magnitude = super::hull::scaled_product(
                &[
                    0.5,
                    flow.atmosphere.density(),
                    speed,
                    speed,
                    cp.abs(),
                    self.area,
                ],
                1.0,
            )
            .map_err(|_| overflow)?;
            let force = normal
                .checked_mul_scalar(-magnitude.copysign(cp))
                .ok_or(overflow)?;
            let moment = arm.checked_cross(force).ok_or(overflow)?;
            return Ok(PressureWrench { force, moment });
        }
        // `relative` is the body velocity through the fluid, pointing from the
        // oncoming fluid toward the body.  Windward patches have outward normals
        // aligned with it; the impact pressure acts back along `-normal`.
        let relative_direction = relative / speed;
        let incidence = normal
            .checked_dot(relative_direction)
            .ok_or(overflow)?
            .max(0.0);
        if incidence == 0.0 {
            return Ok(zero);
        }
        // The factors 1/2 in dynamic pressure and 2 in Newtonian Cp cancel.
        // Scale the complete product so an unrepresentable intermediate pressure
        // does not reject a representable force on a small patch.
        let magnitude = super::hull::scaled_product(
            &[
                flow.atmosphere.density(),
                speed,
                speed,
                incidence,
                incidence,
                self.area,
            ],
            1.0,
        )
        .map_err(|_| overflow)?;
        let force = normal.checked_mul_scalar(-magnitude).ok_or(overflow)?;
        let moment = arm.checked_cross(force).ok_or(overflow)?;
        Ok(PressureWrench { force, moment })
    }
}

/// Sums centroid pressure loads over prepared physical mesh patches.
///
/// `flow` supplies one uniform atmospheric state and sound speed for all patches,
/// a body-to-world transform, body-local COM, and world-space COM/angular velocity.
/// The returned wrench is world-space and its moment is about that COM.
///
/// This allocation-free evaluation preserves `PressurePatch`'s Mach policy and
/// force convention. It does not prepare geometry or update exposure/shielding:
/// callers must select patches for the current body-local incoming direction.
/// Rotation-induced local flow can invalidate that single-direction visibility
/// approximation. No world integration or per-owner load distribution occurs.
/// An error from any patch or a nonfinite sum rejects the entire result.
/// Empty sets return zero after validating motion and sound speed.
pub fn evaluate_prepared_mesh(
    patches: &PreparedSurfacePatches,
    flow: PressureFlow,
) -> Result<PressureWrench, PressureError> {
    evaluate_prepared(patches, flow, None)
}

/// Returns the impact-pressure wrench of one geometry owner about the common
/// body COM, in world coordinates. Geometry ownership reflects the prepared
/// exposure and overlap policy; this is not a complete external or weld load.
/// The result is computed locally and returned only if every selected patch
/// and the accumulated wrench are valid. No body state is mutated.
pub fn evaluate_prepared_mesh_owner(
    patches: &PreparedSurfacePatches,
    flow: PressureFlow,
    owner: super::geometry::GeometryOwner,
) -> Result<PressureWrench, PressureError> {
    let mut total = PressurePatch::new(Vec3::ZERO, Vec3::X, 0.0)?.evaluate(flow)?;
    for patch in patches.iter().filter(|patch| patch.owner == owner) {
        let load = PressurePatch::new(patch.center, patch.normal, patch.area)?.evaluate(flow)?;
        total.force = total
            .force
            .checked_add(load.force)
            .ok_or(PressureError::ArithmeticOverflow)?;
        total.moment = total
            .moment
            .checked_add(load.moment)
            .ok_or(PressureError::ArithmeticOverflow)?;
    }
    Ok(total)
}

/// Evaluates impact-pressure wrenches for all represented owners in one patch
/// traversal. Results are sorted by owner; moments are about the common body
/// center of mass and all vectors are in world coordinates. This is only the
/// impact-pressure contribution, not a complete part or weld load. An error
/// rejects the entire result. Work is O(P log K), storage O(K), for P patches
/// and K represented owners. This allocates and is not a steady-state buffer.
pub fn evaluate_prepared_mesh_by_owner(
    patches: &PreparedSurfacePatches,
    flow: PressureFlow,
) -> Result<Vec<(super::geometry::GeometryOwner, PressureWrench)>, PressureError> {
    let zero = PressurePatch::new(Vec3::ZERO, Vec3::X, 0.0)?.evaluate(flow)?;
    let mut totals = std::collections::BTreeMap::new();
    for patch in patches.iter() {
        let load = PressurePatch::new(patch.center, patch.normal, patch.area)?.evaluate(flow)?;
        let total: &mut PressureWrench = totals.entry(patch.owner).or_insert(zero);
        total.force = total
            .force
            .checked_add(load.force)
            .ok_or(PressureError::ArithmeticOverflow)?;
        total.moment = total
            .moment
            .checked_add(load.moment)
            .ok_or(PressureError::ArithmeticOverflow)?;
    }
    Ok(totals.into_iter().collect())
}

/// Reusable transactional storage for owner-attributed impact-pressure wrenches.
/// `evaluate` replaces published results only on success; errors preserve them.
/// Owners are preconfigured and strictly sorted so patch lookup is O(log K).
/// Once created, evaluation does not allocate. This workspace is not a complete
/// part-load ledger and is not attached to a `PhysicsWorld` step.
pub struct OwnerPressureWorkspace {
    published: Vec<(super::geometry::GeometryOwner, PressureWrench)>,
    scratch: Vec<(super::geometry::GeometryOwner, PressureWrench)>,
    prepared: bool,
}

impl OwnerPressureWorkspace {
    /// Returns `None` for unsorted or repeated owners.
    pub fn new(owners: &[super::geometry::GeometryOwner]) -> Option<Self> {
        if owners.windows(2).any(|pair| pair[0] >= pair[1]) {
            return None;
        }
        let zero = PressureWrench {
            force: Vec3::ZERO,
            moment: Vec3::ZERO,
        };
        let published = owners
            .iter()
            .map(|owner| (*owner, zero))
            .collect::<Vec<_>>();
        Some(Self {
            scratch: published.clone(),
            published,
            prepared: false,
        })
    }

    /// Last successful complete result; initially all configured loads are zero.
    pub fn published(&self) -> &[(super::geometry::GeometryOwner, PressureWrench)] {
        &self.published
    }

    /// Replaces the scratch owner configuration without changing published data.
    /// Invalid order or allocation failure leaves the published result intact and
    /// invalidates any previously prepared result. Existing capacity is reused.
    pub fn configure_pending(
        &mut self,
        owners: &[super::geometry::GeometryOwner],
    ) -> Result<(), PressureError> {
        self.prepared = false;
        if owners.windows(2).any(|pair| pair[0] >= pair[1]) {
            return Err(PressureError::InvalidPatch);
        }
        self.scratch.clear();
        self.scratch
            .try_reserve(owners.len())
            .map_err(|_| PressureError::ArithmeticOverflow)?;
        let zero = PressureWrench {
            force: Vec3::ZERO,
            moment: Vec3::ZERO,
        };
        self.scratch
            .extend(owners.iter().map(|owner| (*owner, zero)));
        Ok(())
    }

    /// Staged owner wrenches, available only after a successful preparation.
    /// These have not yet been published and must be discarded on step failure.
    pub fn staged(&self) -> Option<&[(super::geometry::GeometryOwner, PressureWrench)]> {
        self.prepared.then_some(&self.scratch)
    }

    /// Sums staged owner wrenches about their shared COM, rejecting overflow.
    /// `None` means that there is no successfully staged result.
    pub fn staged_total(&self) -> Result<Option<PressureWrench>, PressureError> {
        let Some(staged) = self.staged() else {
            return Ok(None);
        };
        let mut total = PressureWrench {
            force: Vec3::ZERO,
            moment: Vec3::ZERO,
        };
        for (_, wrench) in staged {
            total.force = total
                .force
                .checked_add(wrench.force)
                .ok_or(PressureError::ArithmeticOverflow)?;
            total.moment = total
                .moment
                .checked_add(wrench.moment)
                .ok_or(PressureError::ArithmeticOverflow)?;
        }
        Ok(Some(total))
    }

    /// Evaluates every patch once; unconfigured owners are rejected.
    pub fn evaluate(
        &mut self,
        patches: &PreparedSurfacePatches,
        flow: PressureFlow,
    ) -> Result<(), PressureError> {
        self.prepare(patches, flow)?;
        self.commit();
        Ok(())
    }

    /// Stages a result without publishing it. A failed preparation invalidates
    /// any earlier staged result while preserving the last published result.
    pub fn prepare(
        &mut self,
        patches: &PreparedSurfacePatches,
        flow: PressureFlow,
    ) -> Result<(), PressureError> {
        self.prepared = false;
        let zero = PressurePatch::new(Vec3::ZERO, Vec3::X, 0.0)?.evaluate(flow)?;
        for (_, wrench) in &mut self.scratch {
            *wrench = zero;
        }
        for patch in patches.iter() {
            let index = self
                .scratch
                .binary_search_by_key(&patch.owner, |entry| entry.0)
                .map_err(|_| PressureError::InvalidPatch)?;
            let load =
                PressurePatch::new(patch.center, patch.normal, patch.area)?.evaluate(flow)?;
            let total = &mut self.scratch[index].1;
            total.force = total
                .force
                .checked_add(load.force)
                .ok_or(PressureError::ArithmeticOverflow)?;
            total.moment = total
                .moment
                .checked_add(load.moment)
                .ok_or(PressureError::ArithmeticOverflow)?;
        }
        self.prepared = true;
        Ok(())
    }

    /// Publishes a successfully staged result once; returns whether one existed.
    pub fn commit(&mut self) -> bool {
        if !self.prepared {
            return false;
        }
        std::mem::swap(&mut self.published, &mut self.scratch);
        self.prepared = false;
        true
    }

    /// Discards a staged result without affecting the published result.
    pub fn rollback(&mut self) {
        self.prepared = false;
    }
}

/// Applies the supplied base Cp to every explicitly selected prepared patch.
/// The set must represent actual exposed base surfaces, not windward impact
/// patches; this function does not determine which surfaces are in a wake.
/// It is independent of whether the patches originated from a mesh or collider.
pub fn evaluate_prepared_base(
    patches: &PreparedSurfacePatches,
    flow: PressureFlow,
    base: BasePressure,
) -> Result<PressureWrench, PressureError> {
    evaluate_prepared(patches, flow, Some(ExplicitPressure::Base(base)))
}

/// Applies an explicitly supplied signed Cp to selected mesh- or collider-derived
/// patches. This does not predict Cp, determine exposure, or model a wake. A
/// single-direction exposure set generally omits suction on other surfaces.
/// No world integration or per-owner load distribution occurs.
pub fn evaluate_prepared_surface(
    patches: &PreparedSurfacePatches,
    flow: PressureFlow,
    pressure: SurfacePressure,
) -> Result<PressureWrench, PressureError> {
    evaluate_prepared(patches, flow, Some(ExplicitPressure::Surface(pressure)))
}

fn evaluate_prepared(
    patches: &PreparedSurfacePatches,
    flow: PressureFlow,
    explicit: Option<ExplicitPressure>,
) -> Result<PressureWrench, PressureError> {
    // Validate even an empty set, using the same rules as patch evaluation.
    let mut total = PressurePatch::new(Vec3::ZERO, Vec3::X, 0.0)?.evaluate(flow)?;
    for patch in patches.iter() {
        let load = PressurePatch::new(patch.center, patch.normal, patch.area)?
            .evaluate_inner(flow, explicit)?;
        total.force = total
            .force
            .checked_add(load.force)
            .ok_or(PressureError::ArithmeticOverflow)?;
        total.moment = total
            .moment
            .checked_add(load.moment)
            .ok_or(PressureError::ArithmeticOverflow)?;
    }
    Ok(total)
}

#[cfg(test)]
#[path = "pressure_tests.rs"]
mod tests;

#[cfg(test)]
#[path = "mesh_pressure_tests.rs"]
mod mesh_tests;
