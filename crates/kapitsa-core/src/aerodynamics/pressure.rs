//! Newtonian hypersonic impact pressure on caller-selected physical surface patches.
//!
//! Source: <https://en.wikipedia.org/wiki/Pressure_coefficient>, Cp = 2 sin²(theta).
//! Leeward excess pressure is zero. Mach >= 5 is an explicit implementation
//! policy, not a universal accuracy guarantee. No suction, friction, wake,
//! circulation, transonic correction or shock interaction is resolved.
//! Callers must supply exposed original-surface normals and areas. In particular,
//! voxel staircase normals are not a consistent replacement for smooth normals.
//! Overlap removal and upstream shielding are the caller's responsibility.
//!
//! [`MeshPressureModel::ModifiedNewtonian`] is a separate opt-in compression
//! model for `0 <= Mach <= 5`. It keeps the same exposed-patch geometry and the
//! impact form `Cp = Cp_max(M) sin²(theta)`, but derives `Cp_max` from
//! isentropic stagnation pressure below Mach 1 and Rayleigh pitot pressure
//! through Mach 5. It is not a potential-flow, lifting, suction, wake or
//! viscous model; leeward pressure remains zero.
//!
//! [`MeshPressureModel::PotentialFlow`] is a bounded incompressible
//! source-panel model for a closed isolated Mesh/Colliders body at local
//! Mach ≤ 0.2 and up to 256 triangles, with quasi-steady rigid rotation but
//! without Kutta wake, circulation or separation.
//! See [`crate::aerodynamics::potential`].
//!
//! [`EmpiricalBasePressure`] is a separate opt-in turbulent-base add-on for a
//! closed primary Mesh/Colliders body at Mach 1–5. It combines Love's
//! cylindrical base-pressure data with MIL-HDBK-762 Fig. 5-141 boattail relief
//! and applies the resulting negative Cp to rear-facing patches within 10
//! degrees of the COM motion direction. It is a bounded empirical wake
//! approximation, not an unsteady wake or cavity-flow solver.

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
    UnsupportedGeometry,
    WorkBudgetExceeded,
    ArithmeticOverflow,
}
impl fmt::Display for PressureError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "surface pressure evaluation failed: {self:?}")
    }
}
impl std::error::Error for PressureError {}

/// Selects the surface-pressure closure used for Mesh/Colliders geometry.
///
/// `Newtonian` is the existing unmodified hypersonic impact model and requires
/// Mach >= 5. `ModifiedNewtonian` is the compression-only extension in
/// `0 <= Mach <= 5`; it still uses exposed windward patches and returns an
/// error above Mach 5. `PotentialFlow` is a bounded incompressible source-panel
/// model for closed isolated meshes at local Mach <= 0.2 with quasi-steady
/// rigid rotation. `KuttaLifting` derives an
/// equivalent finite wing from a closed thin wing mesh and applies its
/// trailing-vortex lifting-line wake. `VortexLattice` applies a bounded
/// horseshoe-vortex wake lattice to a closed thin wing mesh. The default
/// preserves existing behavior.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum MeshPressureModel {
    /// Unmodified Newtonian impact pressure, `Cp = 2 sin²(theta)`, Mach >= 5.
    #[default]
    Newtonian,
    /// Modified Newtonian compression pressure, `Cp = Cp_max(M) sin²(theta)`,
    /// with `Cp_max` from isentropic or Rayleigh-pitot stagnation pressure.
    /// Leeward suction, wake, friction, base flow and circulation are absent.
    ModifiedNewtonian,
    /// Incompressible source-panel potential flow for a closed, isolated
    /// Mesh/Colliders body. Mach ≤ 0.2 and at most 256 mesh triangles.
    /// See [`crate::aerodynamics::potential::evaluate_closed_mesh`].
    PotentialFlow,
    /// Kutta-condition finite-wing lifting-line derived from a closed thin
    /// wing mesh. See
    /// [`crate::aerodynamics::potential::evaluate_kutta_lifting_mesh`].
    KuttaLifting,
    /// Bounded horseshoe-vortex lattice for a closed thin wing mesh.
    /// See [`crate::aerodynamics::vlm::evaluate_closed_wing`].
    VortexLattice,
}

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
) -> Result<(), crate::Error> {
    if selected
        .len()
        .checked_mul(mesh.triangles().len())
        .is_none_or(|work| work > max_work)
    {
        return Err(crate::Error::from(super::AerodynamicsError::Geometry(
            crate::Error::from(super::geometry::GeometryError::WorkBudgetExceeded),
        )));
    }
    for triangle in selected {
        if mesh
            .triangles()
            .iter()
            .filter(|face| same_triangle(triangle, &face.map(|index| mesh.vertices()[index])))
            .count()
            != 1
        {
            return Err(crate::Error::from(super::AerodynamicsError::Pressure(
                crate::Error::from(PressureError::BaseNotFullyVisible),
            )));
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
) -> Result<(), crate::Error> {
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
            return Err(crate::Error::from(PressureError::BaseNotFullyVisible));
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
) -> Result<Vec<PressurePatch>, crate::Error> {
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
            return Err(crate::Error::from(PressureError::InvalidPatch));
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
            .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
        let length = cross
            .checked_length()
            .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
        let center = a / 3.0 + b / 3.0 + c / 3.0;
        patches.push(PressurePatch::new(center, cross, length * 0.5)?);
    }
    let mut neighbors = vec![Vec::new(); faces.len()];
    for edge in edges.values() {
        if edge.len() != 2 || edge[0].1 == edge[1].1 {
            return Err(crate::Error::from(PressureError::InvalidPatch));
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
        return Err(crate::Error::from(PressureError::InvalidPatch));
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
) -> Result<Vec<PressurePatch>, crate::Error> {
    if !center.is_finite()
        || !radius.is_finite()
        || radius <= 0.0
        || !relative_tolerance.is_finite()
        || !(0.0..1.0).contains(&relative_tolerance)
    {
        return Err(crate::Error::from(PressureError::InvalidPatch));
    }
    if mesh
        .triangles()
        .len()
        .checked_mul(mesh.vertices().len())
        .is_none_or(|n| n > max_work)
    {
        return Err(crate::Error::from(PressureError::InvalidPatch));
    }
    let tolerance = radius * relative_tolerance;
    for vertex in mesh.vertices() {
        let distance = vertex
            .checked_sub(center)
            .and_then(|v| v.checked_length())
            .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
        if (distance - radius).abs() > tolerance {
            return Err(crate::Error::from(PressureError::InvalidPatch));
        }
    }
    let patches = closed_mesh_pressure_patches(mesh)?;
    for patch in &patches {
        let offset = patch
            .center
            .checked_sub(center)
            .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
        if patch
            .normal
            .checked_dot(offset)
            .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?
            <= 0.0
        {
            return Err(crate::Error::from(PressureError::InvalidPatch));
        }
        for vertex in mesh.vertices() {
            let delta = vertex
                .checked_sub(patch.center)
                .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
            if patch
                .normal
                .checked_dot(delta)
                .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?
                > tolerance
            {
                return Err(crate::Error::from(PressureError::InvalidPatch));
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
) -> Result<PressureWrench, crate::Error> {
    // Validate flow even for an empty set; an empty set is not a sphere.
    if patches.is_empty() {
        return Err(crate::Error::from(PressureError::InvalidPatch));
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
            .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
        total.moment = total
            .moment
            .checked_add(load.moment)
            .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
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
/// Maximum Mach accepted by the opt-in modified Newtonian compression model.
const MODIFIED_NEWTONIAN_MAX_MACH: f64 = 5.0;
/// Squared surface-speed ratio at the equator of an ideal sphere.
const SPHERE_EQUATORIAL_SPEED_RATIO_SQUARED: f64 = 2.25;
/// Squared surface-speed ratio at the side of an ideal circular cylinder.
const CYLINDER_SIDE_SPEED_RATIO_SQUARED: f64 = 4.0;
/// Dimensionless tolerance for checking perpendicular unit vectors.
const POTENTIAL_CYLINDER_ORTHOGONALITY_TOLERANCE: f64 = 1e-10;
/// Lowest Mach number of the empirical turbulent-base correlation. Below the
/// sonic range the blunt-base pressure depends strongly on geometry, boundary
/// layers and vortex shedding, so it is not extrapolated.
const EMPIRICAL_BASE_MIN_MACH: f64 = 1.0;
/// Highest Mach number of the tabulated Love base-pressure data.
const EMPIRICAL_BASE_MAX_MACH: f64 = 5.0;
/// Lowest Mach number of MIL-HDBK-762 Fig. 5-141's boattail base-relief
/// correlation. The relief factor is held from Mach 1 to this value.
const EMPIRICAL_BASE_RELIEF_MIN_MACH: f64 = 2.5;
/// Ratio of specific heats used by the pressure conversion in the base
/// correlation.
const EMPIRICAL_BASE_GAMMA: f64 = 1.4;
/// A rear patch is treated as base area only when its outward normal is
/// within this angle of the oncoming body-motion direction. Larger angles are
/// aft skin or boattail surface, not a blunt base.
const EMPIRICAL_BASE_MAX_OFF_AXIS_ANGLE: f64 = 10.0 * std::f64::consts::PI / 180.0;
/// Love's turbulent base-pressure magnitude `-Cp_b` behind a cylindrical
/// afterbody against Mach number (MIL-HDBK-762 Fig. 5-139, printed p. 5-208,
/// after NACA TN 3819). Values are held at the endpoints.
const LOVE_BASE_PRESSURE: [(f64, f64); 6] = [
    (2.5, 0.1195),
    (3.0, 0.097),
    (3.5, 0.0805),
    (4.0, 0.067),
    (4.5, 0.0565),
    (5.0, 0.0475),
];

impl SurfacePressure {
    pub fn new(cp: f64, min_mach: f64, max_mach: f64) -> Result<Self, crate::Error> {
        if !cp.is_finite()
            || !min_mach.is_finite()
            || min_mach < 0.0
            || !max_mach.is_finite()
            || max_mach < min_mach
        {
            return Err(crate::Error::from(PressureError::InvalidCoefficient));
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
    pub fn new(cp: f64, min_mach: f64, max_mach: f64) -> Result<Self, crate::Error> {
        if !cp.is_finite()
            || cp > 0.0
            || !min_mach.is_finite()
            || min_mach < 0.0
            || !max_mach.is_finite()
            || max_mach < min_mach
        {
            return Err(crate::Error::from(PressureError::InvalidCoefficient));
        }
        Ok(Self {
            cp_magnitude: -cp,
            min_mach,
            max_mach,
        })
    }
}

/// Bounded empirical base pressure for a turbulent cylindrical afterbody with
/// MIL-HDBK-762 boattail relief.
///
/// This is not a wake solver and does not infer base exposure. It supplies a
/// Mach- and area-ratio-dependent coefficient for rear-facing prepared patches;
/// [`evaluate_prepared_mesh_with_empirical_base`] performs the geometric
/// selection. The domain is Mach 1–5. Between Mach 1 and 2.5 the correlations
/// are held at their Mach 2.5 values because the available tabulation begins
/// there; this is an explicit engineering hold, not a transonic validation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EmpiricalBasePressure {
    base_area_ratio: f64,
}

impl EmpiricalBasePressure {
    /// `base_area_ratio` is base area divided by the body's maximum
    /// cross-sectional (forebody) area, in `(0, 1]`. The caller must establish
    /// that ratio and the applicability of turbulent base flow.
    pub fn new(base_area_ratio: f64) -> Result<Self, crate::Error> {
        if !base_area_ratio.is_finite() || base_area_ratio <= 0.0 || base_area_ratio > 1.0 {
            return Err(crate::Error::from(PressureError::InvalidCoefficient));
        }
        Ok(Self { base_area_ratio })
    }

    pub fn base_area_ratio(self) -> f64 {
        self.base_area_ratio
    }

    /// Returns the base pressure coefficient `Cp_b <= 0` at `mach`.
    ///
    /// `Cp_b = -(-Cp_cyl(M)) k(M, a_b)`, where `-Cp_cyl` is Love's cylindrical
    /// base-pressure data and `k` is MIL-HDBK-762 Fig. 5-141's base-pressure
    /// relief ratio. Outside Mach 1–5 this returns
    /// [`PressureError::UnsupportedMach`].
    pub fn coefficient(self, mach: f64) -> Result<f64, crate::Error> {
        if !mach.is_finite() || !(EMPIRICAL_BASE_MIN_MACH..=EMPIRICAL_BASE_MAX_MACH).contains(&mach)
        {
            return Err(crate::Error::from(PressureError::UnsupportedMach));
        }
        let cylinder = love_base_pressure(mach.max(EMPIRICAL_BASE_RELIEF_MIN_MACH));
        let relief = self.relief_ratio(mach.max(EMPIRICAL_BASE_RELIEF_MIN_MACH));
        let coefficient = -(cylinder * relief);
        if !coefficient.is_finite() {
            return Err(crate::Error::from(PressureError::ArithmeticOverflow));
        }
        Ok(coefficient)
    }

    fn relief_ratio(self, mach: f64) -> f64 {
        // p_cyl/p from Love's coefficient; p_bt/p from MIL-HDBK-762
        // Fig. 5-141, then k = Cp_bt / Cp_cyl in pressure terms.
        let cylinder = love_base_pressure(mach);
        let p_cylinder = (1.0 - cylinder * 0.5 * EMPIRICAL_BASE_GAMMA * mach * mach).max(0.0);
        let p_base = p_cylinder / (0.442 + 0.558 * self.base_area_ratio);
        ((1.0 - p_base) / (1.0 - p_cylinder)).clamp(0.0, 1.0)
    }
}

/// Love's `-Cp_b` for a cylinder, linearly interpolated between the tabulated
/// readings and held at their endpoints.
fn love_base_pressure(mach: f64) -> f64 {
    let first = LOVE_BASE_PRESSURE[0];
    let last = LOVE_BASE_PRESSURE[LOVE_BASE_PRESSURE.len() - 1];
    if mach <= first.0 {
        return first.1;
    }
    if mach >= last.0 {
        return last.1;
    }
    let index = LOVE_BASE_PRESSURE.partition_point(|point| point.0 <= mach) - 1;
    let (m0, c0) = LOVE_BASE_PRESSURE[index];
    let (m1, c1) = LOVE_BASE_PRESSURE[index + 1];
    c0 + (c1 - c0) * (mach - m0) / (m1 - m0)
}

impl PressurePatch {
    pub fn new(center: Vec3, outward_normal: Vec3, area: f64) -> Result<Self, crate::Error> {
        if !center.is_finite() || !area.is_finite() || area < 0.0 {
            return Err(crate::Error::from(PressureError::InvalidPatch));
        }
        let normal = outward_normal
            .try_normalize()
            .ok_or_else(|| crate::Error::from(PressureError::InvalidPatch))?;
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
    ) -> Result<PressureWrench, crate::Error> {
        if flow.angular_velocity != Vec3::ZERO {
            return Err(crate::Error::from(PressureError::InvalidMotion));
        }
        // Reuse the ordinary checks even for a zero-speed or zero-area patch.
        let zero = SurfacePressure::new(0.0, 0.0, POTENTIAL_FLOW_MAX_MACH)?;
        self.evaluate_surface(flow, zero)?;
        let relative = flow
            .linear_velocity
            .checked_sub(flow.atmosphere.wind_velocity())
            .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
        let speed = relative
            .checked_length()
            .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
        if speed == 0.0 || self.area == 0.0 || flow.atmosphere.density() == 0.0 {
            return Ok(PressureWrench {
                force: Vec3::ZERO,
                moment: Vec3::ZERO,
            });
        }
        let normal = flow.transform.transform_vector(self.normal);
        let cosine = normal
            .checked_dot(relative / speed)
            .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?
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
    /// Uniform flow may have an axial component, which contributes no lateral
    /// pressure in this inviscid model. Angular velocity is unsupported. The
    /// Mach <= 0.2 limit applies to the total relative speed and is an
    /// approximation policy, not an experimental accuracy guarantee. The
    /// crossflow coefficient is 1 - 4 sin²(theta), measured from a stagnation
    /// point and referenced to crossflow dynamic pressure.
    pub fn evaluate_potential_cylinder(
        &self,
        flow: PressureFlow,
        axis: Vec3,
    ) -> Result<PressureWrench, crate::Error> {
        let axis = axis
            .try_normalize()
            .ok_or_else(|| crate::Error::from(PressureError::InvalidPatch))?;
        if self
            .normal
            .checked_dot(axis)
            .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?
            .abs()
            > POTENTIAL_CYLINDER_ORTHOGONALITY_TOLERANCE
        {
            return Err(crate::Error::from(PressureError::InvalidPatch));
        }
        if flow.angular_velocity != Vec3::ZERO {
            return Err(crate::Error::from(PressureError::InvalidMotion));
        }
        let zero = SurfacePressure::new(0.0, 0.0, POTENTIAL_FLOW_MAX_MACH)?;
        let zero_load = self.evaluate_surface(flow, zero)?;
        let relative = flow
            .linear_velocity
            .checked_sub(flow.atmosphere.wind_velocity())
            .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
        let speed = relative
            .checked_length()
            .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
        if speed == 0.0 || self.area == 0.0 || flow.atmosphere.density() == 0.0 {
            return Ok(zero_load);
        }
        let direction = relative / speed;
        let world_axis = flow.transform.transform_vector(axis);
        let axial = world_axis
            .checked_dot(direction)
            .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
        let crossflow = direction
            .checked_sub(
                world_axis
                    .checked_mul_scalar(axial)
                    .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?,
            )
            .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
        let normal = flow.transform.transform_vector(self.normal);
        let normal_speed = normal
            .checked_dot(crossflow)
            .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
        let crossflow_squared = crossflow
            .checked_dot(crossflow)
            .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
        if crossflow != Vec3::ZERO && crossflow_squared == 0.0 {
            return Err(crate::Error::from(PressureError::ArithmeticOverflow));
        }
        // Cp is referenced to total dynamic pressure by evaluate_surface.
        let cp = crossflow_squared
            - CYLINDER_SIDE_SPEED_RATIO_SQUARED * (crossflow_squared - normal_speed * normal_speed);
        self.evaluate_surface(
            flow,
            SurfacePressure::new(cp, 0.0, POTENTIAL_FLOW_MAX_MACH)?,
        )
    }

    /// Evaluates a centroid load sample; supply atmosphere sampled at this patch.
    /// COM is body-local; velocities, force and moment are world-space.
    /// No allocation, geometry preparation or body mutation occurs here.
    /// This is the unmodified Newtonian model and retains its Mach >= 5 domain.
    pub fn evaluate(&self, flow: PressureFlow) -> Result<PressureWrench, crate::Error> {
        self.evaluate_inner(flow, None, MeshPressureModel::Newtonian)
    }

    /// Modified Newtonian compression pressure for `0 <= Mach <= 5`.
    ///
    /// The geometry and exposure conventions are identical to [`Self::evaluate`].
    /// `Cp = Cp_max(M) sin²(theta)` with isentropic stagnation pressure below
    /// Mach 1 and Rayleigh pitot pressure from Mach 1 to 5. Air is assumed to be
    /// calorically perfect with `gamma = 1.4`. Leeward pressure, suction, wake,
    /// friction, base flow and circulation are not modeled. A finite Mach above
    /// 5 returns [`PressureError::UnsupportedMach`]; select the unmodified model
    /// for hypersonic flow instead.
    pub fn evaluate_modified_newtonian(
        &self,
        flow: PressureFlow,
    ) -> Result<PressureWrench, crate::Error> {
        self.evaluate_inner(flow, None, MeshPressureModel::ModifiedNewtonian)
    }

    pub(crate) fn evaluate_with_model(
        &self,
        flow: PressureFlow,
        model: MeshPressureModel,
    ) -> Result<PressureWrench, crate::Error> {
        self.evaluate_inner(flow, None, model)
    }

    /// Applies a caller-supplied constant base Cp to this explicitly selected
    /// physical patch. Force acts outward for negative Cp. A patch whose outward
    /// normal points into its local body-relative velocity is rejected; callers
    /// must still establish that the patch is a base and is not shielded.
    pub fn evaluate_base(
        &self,
        flow: PressureFlow,
        base: BasePressure,
    ) -> Result<PressureWrench, crate::Error> {
        self.evaluate_inner(
            flow,
            Some(ExplicitPressure::Base(base)),
            MeshPressureModel::Newtonian,
        )
    }

    /// Applies a supplied signed Cp without inferring exposure or wake physics.
    /// Pressure acts along the inward normal for positive Cp and outward for negative Cp.
    pub fn evaluate_surface(
        &self,
        flow: PressureFlow,
        pressure: SurfacePressure,
    ) -> Result<PressureWrench, crate::Error> {
        self.evaluate_inner(
            flow,
            Some(ExplicitPressure::Surface(pressure)),
            MeshPressureModel::Newtonian,
        )
    }

    fn evaluate_inner(
        &self,
        flow: PressureFlow,
        explicit: Option<ExplicitPressure>,
        model: MeshPressureModel,
    ) -> Result<PressureWrench, crate::Error> {
        if matches!(
            model,
            MeshPressureModel::PotentialFlow
                | MeshPressureModel::KuttaLifting
                | MeshPressureModel::VortexLattice
        ) {
            return Err(crate::Error::from(PressureError::UnsupportedGeometry));
        }
        if !flow.center_of_mass.is_finite()
            || !flow.linear_velocity.is_finite()
            || !flow.angular_velocity.is_finite()
        {
            return Err(crate::Error::from(PressureError::InvalidMotion));
        }
        if !flow.sound_speed.is_finite() || flow.sound_speed <= 0.0 {
            return Err(crate::Error::from(PressureError::InvalidSoundSpeed));
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
            return Err(crate::Error::from(overflow));
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
            return Err(crate::Error::from(overflow));
        }
        let domain = match explicit {
            Some(ExplicitPressure::Base(b)) => Some((b.min_mach, b.max_mach)),
            Some(ExplicitPressure::Surface(p)) => Some((p.min_mach, p.max_mach)),
            None => None,
        };
        if let Some((min, max)) = domain {
            if mach < min || mach > max {
                return Err(crate::Error::from(PressureError::UnsupportedMach));
            }
        } else {
            match model {
                MeshPressureModel::Newtonian if mach < 5.0 => {
                    return Err(crate::Error::from(PressureError::UnsupportedMach));
                }
                MeshPressureModel::ModifiedNewtonian if mach > MODIFIED_NEWTONIAN_MAX_MACH => {
                    return Err(crate::Error::from(PressureError::UnsupportedMach));
                }
                _ => {}
            }
        }
        if let Some(explicit) = explicit {
            let cp = match explicit {
                ExplicitPressure::Base(base) => {
                    let alignment = normal.checked_dot(relative / speed).ok_or(overflow)?;
                    if alignment > 0.0 {
                        return Err(crate::Error::from(PressureError::InvalidBaseOrientation));
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
        let magnitude = match model {
            MeshPressureModel::Newtonian => super::hull::scaled_product(
                &[
                    flow.atmosphere.density(),
                    speed,
                    speed,
                    incidence,
                    incidence,
                    self.area,
                ],
                1.0,
            ),
            MeshPressureModel::ModifiedNewtonian => {
                let cp_max = modified_newtonian_stagnation_cp(mach)?;
                super::hull::scaled_product(
                    &[
                        0.5,
                        flow.atmosphere.density(),
                        speed,
                        speed,
                        cp_max,
                        incidence,
                        incidence,
                        self.area,
                    ],
                    1.0,
                )
            }
            MeshPressureModel::PotentialFlow
            | MeshPressureModel::KuttaLifting
            | MeshPressureModel::VortexLattice => {
                return Err(crate::Error::from(PressureError::UnsupportedGeometry));
            }
        }
        .map_err(|_| overflow)?;
        let force = normal.checked_mul_scalar(-magnitude).ok_or(overflow)?;
        let moment = arm.checked_cross(force).ok_or(overflow)?;
        Ok(PressureWrench { force, moment })
    }
}

/// Stagnation pressure coefficient for calorically perfect air (`gamma = 1.4`).
///
/// Below Mach 1 this is the isentropic stagnation value; from Mach 1 through 5
/// it is the Rayleigh pitot value behind a normal shock. At `M = 0` the limit is
/// one. The caller validates the Mach domain before calling this helper.
fn modified_newtonian_stagnation_cp(mach: f64) -> Result<f64, crate::Error> {
    const AIR_GAMMA: f64 = 1.4;
    const EXPONENT: f64 = AIR_GAMMA / (AIR_GAMMA - 1.0);
    if !mach.is_finite() || mach < 0.0 {
        return Err(crate::Error::from(PressureError::UnsupportedMach));
    }
    if mach == 0.0 {
        return Ok(1.0);
    }
    let mach_squared = mach * mach;
    let stagnation_over_static = if mach < 1.0 {
        (1.0 + 0.5 * (AIR_GAMMA - 1.0) * mach_squared).powf(EXPONENT)
    } else {
        let shock_denominator = 2.0 * AIR_GAMMA * mach_squared - (AIR_GAMMA - 1.0);
        if !shock_denominator.is_finite() || shock_denominator <= 0.0 {
            return Err(crate::Error::from(PressureError::ArithmeticOverflow));
        }
        let downstream_mach_squared = ((AIR_GAMMA - 1.0) * mach_squared + 2.0) / shock_denominator;
        let static_ratio = shock_denominator / (AIR_GAMMA + 1.0);
        let total_ratio = (1.0 + 0.5 * (AIR_GAMMA - 1.0) * downstream_mach_squared).powf(EXPONENT);
        static_ratio * total_ratio
    };
    let dynamic_over_static = 0.5 * AIR_GAMMA * mach_squared;
    let coefficient = (stagnation_over_static - 1.0) / dynamic_over_static;
    coefficient
        .is_finite()
        .then_some(coefficient)
        .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))
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
) -> Result<PressureWrench, crate::Error> {
    evaluate_prepared(patches, flow, None, MeshPressureModel::Newtonian)
}

/// Evaluates the opt-in modified Newtonian compression model over prepared
/// patches, in world coordinates and about the common body COM.
///
/// The model is valid for `0 <= Mach <= 5` and uses the same exposure and
/// overlap geometry as [`evaluate_prepared_mesh`]. It is compression-only:
/// windward patches receive `Cp = Cp_max(M) sin²(theta)` and leeward patches
/// receive zero. This is not a potential-flow or wake solver, and it does not
/// add suction, friction, base pressure or circulation. A finite Mach above 5
/// returns [`PressureError::UnsupportedMach`]. Empty sets return zero after
/// validating motion and sound speed. No allocation or geometry mutation occurs.
pub fn evaluate_prepared_mesh_modified_newtonian(
    patches: &PreparedSurfacePatches,
    flow: PressureFlow,
) -> Result<PressureWrench, crate::Error> {
    evaluate_prepared(patches, flow, None, MeshPressureModel::ModifiedNewtonian)
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
) -> Result<PressureWrench, crate::Error> {
    evaluate_prepared_mesh_owner_with_model(patches, flow, owner, MeshPressureModel::Newtonian)
}

/// Model-selectable variant of [`evaluate_prepared_mesh_owner`].
pub fn evaluate_prepared_mesh_owner_with_model(
    patches: &PreparedSurfacePatches,
    flow: PressureFlow,
    owner: super::geometry::GeometryOwner,
    model: MeshPressureModel,
) -> Result<PressureWrench, crate::Error> {
    let mut total =
        PressurePatch::new(Vec3::ZERO, Vec3::X, 0.0)?.evaluate_with_model(flow, model)?;
    for patch in patches.iter().filter(|patch| patch.owner == owner) {
        let load = PressurePatch::new(patch.center, patch.normal, patch.area)?
            .evaluate_with_model(flow, model)?;
        total.force = total
            .force
            .checked_add(load.force)
            .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
        total.moment = total
            .moment
            .checked_add(load.moment)
            .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
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
) -> Result<Vec<(super::geometry::GeometryOwner, PressureWrench)>, crate::Error> {
    evaluate_prepared_mesh_by_owner_with_model(patches, flow, MeshPressureModel::Newtonian)
}

/// Model-selectable variant of [`evaluate_prepared_mesh_by_owner`].
pub fn evaluate_prepared_mesh_by_owner_with_model(
    patches: &PreparedSurfacePatches,
    flow: PressureFlow,
    model: MeshPressureModel,
) -> Result<Vec<(super::geometry::GeometryOwner, PressureWrench)>, crate::Error> {
    let zero = PressurePatch::new(Vec3::ZERO, Vec3::X, 0.0)?.evaluate_with_model(flow, model)?;
    let mut totals = std::collections::BTreeMap::new();
    for patch in patches.iter() {
        let load = PressurePatch::new(patch.center, patch.normal, patch.area)?
            .evaluate_with_model(flow, model)?;
        let total: &mut PressureWrench = totals.entry(patch.owner).or_insert(zero);
        total.force = total
            .force
            .checked_add(load.force)
            .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
        total.moment = total
            .moment
            .checked_add(load.moment)
            .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
    }
    Ok(totals.into_iter().collect())
}

/// Reusable transactional storage for owner-attributed impact-pressure wrenches.
/// `evaluate` replaces published results only on success; errors preserve them.
/// Owners are preconfigured and strictly sorted so patch lookup is O(log K).
/// Once created, evaluation does not allocate. This workspace alone is not a
/// complete part-load ledger; the world stages it transactionally when needed.
pub struct OwnerPressureWorkspace {
    published: Vec<(super::geometry::GeometryOwner, PressureWrench)>,
    scratch: Vec<(super::geometry::GeometryOwner, PressureWrench)>,
    pressure_model: MeshPressureModel,
    prepared: bool,
    scratch_total: PressureWrench,
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
            pressure_model: MeshPressureModel::Newtonian,
            prepared: false,
            scratch_total: zero,
        })
    }

    /// Selects the closure used by subsequent preparations. Changing the model
    /// invalidates any staged result but preserves the last published result.
    pub fn set_pressure_model(&mut self, model: MeshPressureModel) {
        if self.pressure_model != model {
            self.prepared = false;
            self.pressure_model = model;
        }
    }

    /// Current closure used by [`Self::prepare`].
    pub fn pressure_model(&self) -> MeshPressureModel {
        self.pressure_model
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
    ) -> Result<(), crate::Error> {
        self.prepared = false;
        if owners.windows(2).any(|pair| pair[0] >= pair[1]) {
            return Err(crate::Error::from(PressureError::InvalidPatch));
        }
        self.scratch.clear();
        self.scratch
            .try_reserve(owners.len())
            .map_err(|_| crate::Error::from(PressureError::ArithmeticOverflow))?;
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

    /// Returns the staged total about the shared COM, accumulated in patch order.
    /// `None` means that there is no successfully staged result.
    pub fn staged_total(&self) -> Result<Option<PressureWrench>, crate::Error> {
        Ok(self.prepared.then_some(self.scratch_total))
    }

    /// Evaluates every patch once; unconfigured owners are rejected.
    pub fn evaluate(
        &mut self,
        patches: &PreparedSurfacePatches,
        flow: PressureFlow,
    ) -> Result<(), crate::Error> {
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
    ) -> Result<(), crate::Error> {
        self.prepared = false;
        let zero = PressurePatch::new(Vec3::ZERO, Vec3::X, 0.0)?
            .evaluate_with_model(flow, self.pressure_model)?;
        self.scratch_total = zero;
        for (_, wrench) in &mut self.scratch {
            *wrench = zero;
        }
        for patch in patches.iter() {
            let index = self
                .scratch
                .binary_search_by_key(&patch.owner, |entry| entry.0)
                .map_err(|_| crate::Error::from(PressureError::InvalidPatch))?;
            let load = PressurePatch::new(patch.center, patch.normal, patch.area)?
                .evaluate_with_model(flow, self.pressure_model)?;
            self.scratch_total.force = self
                .scratch_total
                .force
                .checked_add(load.force)
                .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
            self.scratch_total.moment = self
                .scratch_total
                .moment
                .checked_add(load.moment)
                .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
            let total = &mut self.scratch[index].1;
            total.force = total
                .force
                .checked_add(load.force)
                .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
            total.moment = total
                .moment
                .checked_add(load.moment)
                .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
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
) -> Result<PressureWrench, crate::Error> {
    evaluate_prepared(
        patches,
        flow,
        Some(ExplicitPressure::Base(base)),
        MeshPressureModel::Newtonian,
    )
}

/// Applies an explicitly supplied signed Cp to selected mesh- or collider-derived
/// patches. This does not predict Cp, determine exposure, or model a wake. A
/// single-direction exposure set generally omits suction on other surfaces.
/// No world integration or per-owner load distribution occurs.
pub fn evaluate_prepared_surface(
    patches: &PreparedSurfacePatches,
    flow: PressureFlow,
    pressure: SurfacePressure,
) -> Result<PressureWrench, crate::Error> {
    evaluate_prepared(
        patches,
        flow,
        Some(ExplicitPressure::Surface(pressure)),
        MeshPressureModel::Newtonian,
    )
}

/// Adds an empirical base-pressure load to a prepared Mesh/Colliders body.
///
/// The surface contribution uses [`evaluate_prepared_mesh`] or
/// [`evaluate_prepared_mesh_modified_newtonian`] through `model`; only those
/// two compression closures are supported. Rear-facing patches whose outward
/// normal is within 10 degrees of the body-motion direction
/// (`EMPIRICAL_BASE_MAX_OFF_AXIS_ANGLE`) receive `Cp_b(M, a_b)` from `base`.
/// The load is summed about the common body COM with the same world-space
/// conventions as the surface loads.
///
/// Base classification uses the COM translation relative to the atmosphere
/// wind. Rotation is not used to reclassify or locally vary the base pressure;
/// this is a bounded empirical wake approximation, not a solution of the
/// separated wake. The model requires at least one qualifying base patch and a
/// Mach number in 1–5 (except exactly zero relative speed, which contributes no
/// base load). The caller must prepare exposed patches and establish that the
/// geometry has a turbulent blunt base and that `base_area_ratio` is applicable.
pub fn evaluate_prepared_mesh_with_empirical_base(
    patches: &PreparedSurfacePatches,
    flow: PressureFlow,
    model: MeshPressureModel,
    base: EmpiricalBasePressure,
) -> Result<PressureWrench, crate::Error> {
    if matches!(
        model,
        MeshPressureModel::PotentialFlow
            | MeshPressureModel::KuttaLifting
            | MeshPressureModel::VortexLattice
    ) {
        return Err(crate::Error::from(PressureError::UnsupportedGeometry));
    }
    let mut total = evaluate_prepared(patches, flow, None, model)?;
    add_empirical_base(
        patches
            .iter()
            .map(|patch| (patch.center, patch.normal, patch.area)),
        flow,
        base,
        &mut total,
    )?;
    Ok(total)
}

/// Adds an empirical base load to `total` from already validated body-local
/// patches. The caller must have validated motion, sound speed and model domain.
fn add_empirical_base(
    patches: impl Iterator<Item = (Vec3, Vec3, f64)>,
    flow: PressureFlow,
    base: EmpiricalBasePressure,
    total: &mut PressureWrench,
) -> Result<(), crate::Error> {
    let overflow = PressureError::ArithmeticOverflow;
    let relative = flow
        .linear_velocity
        .checked_sub(flow.atmosphere.wind_velocity())
        .ok_or(overflow)?;
    let speed = relative.checked_length().ok_or(overflow)?;
    if speed == 0.0 {
        return Ok(());
    }
    let mach = speed / flow.sound_speed;
    let coefficient = base.coefficient(mach)?;
    let direction = (relative / speed)
        .try_normalize()
        .ok_or_else(|| crate::Error::from(PressureError::InvalidMotion))?;
    let threshold = EMPIRICAL_BASE_MAX_OFF_AXIS_ANGLE.cos();
    let mut base_area = 0.0;
    for (center, raw_normal, area) in patches {
        let normal = flow.transform.transform_vector(raw_normal);
        if !normal.is_finite() {
            return Err(crate::Error::from(overflow));
        }
        let alignment = normal.checked_dot(direction).ok_or(overflow)?;
        if alignment > -threshold {
            continue;
        }
        let offset = center.checked_sub(flow.center_of_mass).ok_or(overflow)?;
        let arm = flow.transform.transform_vector(offset);
        if !arm.is_finite() {
            return Err(crate::Error::from(overflow));
        }
        let magnitude = super::hull::scaled_product(
            &[
                0.5,
                flow.atmosphere.density(),
                speed,
                speed,
                coefficient.abs(),
                area,
            ],
            1.0,
        )?;
        let force = normal.checked_mul_scalar(magnitude).ok_or(overflow)?;
        let moment = arm.checked_cross(force).ok_or(overflow)?;
        base_area += area;
        if !base_area.is_finite() {
            return Err(crate::Error::from(overflow));
        }
        total.force = total.force.checked_add(force).ok_or(overflow)?;
        total.moment = total.moment.checked_add(moment).ok_or(overflow)?;
    }
    if base_area == 0.0 {
        return Err(crate::Error::from(PressureError::UnsupportedGeometry));
    }
    Ok(())
}

/// Evaluates only the empirical base-pressure contribution over body-local
/// patches, such as the closed-mesh triangle list returned by
/// [`closed_mesh_pressure_patches`].
///
/// The same Mach/geometry domain and classification as
/// [`evaluate_prepared_mesh_with_empirical_base`] apply. The result is a
/// world-space wrench about the common body COM and contains no surface
/// pressure, so callers add it to their chosen surface model.
pub fn evaluate_empirical_base_patches(
    patches: &[PressurePatch],
    flow: PressureFlow,
    base: EmpiricalBasePressure,
) -> Result<PressureWrench, crate::Error> {
    let mut total = PressurePatch::new(Vec3::ZERO, Vec3::X, 0.0)?
        .evaluate_with_model(flow, MeshPressureModel::ModifiedNewtonian)?;
    add_empirical_base(
        patches
            .iter()
            .map(|patch| (patch.center(), patch.normal(), patch.area())),
        flow,
        base,
        &mut total,
    )?;
    Ok(total)
}

fn evaluate_prepared(
    patches: &PreparedSurfacePatches,
    flow: PressureFlow,
    explicit: Option<ExplicitPressure>,
    model: MeshPressureModel,
) -> Result<PressureWrench, crate::Error> {
    // Validate even an empty set, using the same rules as patch evaluation.
    let mut total =
        PressurePatch::new(Vec3::ZERO, Vec3::X, 0.0)?.evaluate_with_model(flow, model)?;
    for patch in patches.iter() {
        let load = PressurePatch::new(patch.center, patch.normal, patch.area)?
            .evaluate_inner(flow, explicit, model)?;
        total.force = total
            .force
            .checked_add(load.force)
            .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
        total.moment = total
            .moment
            .checked_add(load.moment)
            .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
    }
    Ok(total)
}

#[cfg(test)]
#[path = "pressure_tests.rs"]
mod tests;

#[cfg(test)]
#[path = "mesh_pressure_tests.rs"]
mod mesh_tests;
