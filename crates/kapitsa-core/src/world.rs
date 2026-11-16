use kapitsa_math::{Affine3, Vec3};
use std::{
    collections::HashMap,
    panic::{AssertUnwindSafe, catch_unwind},
    sync::atomic::{AtomicU64, Ordering},
};

#[cfg(feature = "bench-internals")]
use crate::collision::ContactManifold;
#[cfg(feature = "bench-internals")]
use std::time::Instant;

mod aerodynamics {
    use super::{BodyId, PartId, PhysicsWorld, StepError};
    use crate::aerodynamics::{AerodynamicProperties, Atmosphere};

    /// Identifies the rigid object that owns an aerodynamic surface.
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub enum AerodynamicTarget {
        Body,
        Part(PartId),
    }

    /// Aerodynamic properties attached to a body or one of its cluster parts.
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub struct AerodynamicSurface {
        properties: AerodynamicProperties,
        target: AerodynamicTarget,
    }

    impl AerodynamicSurface {
        pub fn new(properties: AerodynamicProperties) -> Self {
            Self {
                properties,
                target: AerodynamicTarget::Body,
            }
        }

        pub fn for_part(part: PartId, properties: AerodynamicProperties) -> Self {
            Self {
                properties,
                target: AerodynamicTarget::Part(part),
            }
        }

        pub fn properties(self) -> AerodynamicProperties {
            self.properties
        }

        pub fn target(self) -> AerodynamicTarget {
            self.target
        }

        pub(crate) fn with_properties(self, properties: AerodynamicProperties) -> Self {
            Self { properties, ..self }
        }
    }

    /// Checks the current aerodynamic force calculation without advancing the world.
    ///
    /// This preflight does not apply forces or check timestep-dependent integration.
    /// Bodies with base-pressure patches return `UnsupportedModel`: this path
    /// does not prepare the primary geometry needed to verify their visibility.
    /// Use [`PhysicsWorld::step`] to advance the simulation.
    #[derive(Clone, Copy, Debug, Default)]
    pub struct AerodynamicsTask;

    impl AerodynamicsTask {
        pub fn execute(self, world: &PhysicsWorld) -> Result<(), StepError> {
            let atmosphere = world.atmosphere;
            if matches!(atmosphere, Atmosphere::Vacuum) {
                return Ok(());
            };
            let mut assembly_cache = None;
            for (dense, entry) in world.bodies.iter() {
                if entry.body.aerodynamic_surfaces.is_empty()
                    && entry.body.base_pressure_patches.is_empty()
                    && !super::assembly_aerodynamics::wings_configured(entry, &world.parts)
                    && !super::assembly_aerodynamics::hull_configured(entry, &world.parts)
                    && matches!(
                        entry.body.aerodynamics(),
                        crate::aerodynamics::Aerodynamics::None
                    )
                    && !super::assembly_aerodynamics::configured(
                        entry,
                        world.part_aerodynamic_geometry.as_ref(),
                    )
                {
                    continue;
                }
                let body_id = BodyId {
                    world: world.world_id,
                    dense,
                };
                if !entry.body.base_pressure_patches.is_empty()
                    || entry.body.potential_sphere.is_some()
                {
                    return Err(StepError::Aerodynamics {
                        body_id,
                        cause: crate::aerodynamics::AerodynamicsError::UnsupportedModel,
                    });
                }
                let patches = super::assembly_aerodynamics::prepare(
                    body_id,
                    entry,
                    &world.parts,
                    world.part_aerodynamic_geometry.as_ref(),
                    &world.colliders,
                    &mut assembly_cache,
                    atmosphere,
                )?;
                let mut body = entry.body.motion();
                let mut wave_cache = crate::aerodynamics::cache::PreparedGeometryCache::default();
                let voxel_wave = if let crate::aerodynamics::Aerodynamics::VoxelWave(model) =
                    entry.body.aerodynamics()
                {
                    PhysicsWorld::prepare_voxel_wave(
                        model,
                        body_id,
                        &body,
                        atmosphere,
                        &mut wave_cache,
                    )?
                } else {
                    None
                };
                PhysicsWorld::apply_aerodynamics(
                    atmosphere,
                    entry.body.aerodynamics(),
                    &entry.body.aerodynamic_surfaces,
                    BodyId {
                        world: world.world_id,
                        dense,
                    },
                    &mut body,
                    &world.parts,
                    super::AerodynamicsInputs::new(world.mesh_sound_speed)
                        .with_base_pressure(&entry.body.base_pressure_patches)
                        .with_cluster(entry.cluster.as_ref())
                        .with_assembly(patches.as_ref())
                        .with_voxel_wave(voxel_wave.as_ref()),
                )?;
            }
            Ok(())
        }
    }
}
#[cfg(test)]
mod aerodynamics_tests;
mod assembly_aerodynamics;
mod ccd;
mod chunk;
mod cluster;
mod error;
#[cfg(test)]
mod gravity_fields_tests;
mod id;
#[cfg(feature = "bench-internals")]
mod metrics;
#[cfg(test)]
mod part_hull_tests;
#[cfg(test)]
mod part_wings_tests;
mod query;

pub use aerodynamics::{AerodynamicSurface, AerodynamicTarget, AerodynamicsTask};
pub use chunk::{AddChunkBatchError, AddChunkError, ChunkOperationError, RemoveChunkBatchError};
pub use error::{ClusterError, ForceSourceError, StepError, WorldError};
pub use id::{BodyId, ChunkId, ColliderId, ForceSourceId, GravityId, JointId, PartId, WeldId};
#[cfg(feature = "bench-internals")]
pub use metrics::PhysicsStepMetrics;

static NEXT_WORLD_ID: AtomicU64 = AtomicU64::new(1);

fn allocate_world_id(counter: &AtomicU64) -> Result<u64, WorldError> {
    counter
        .try_update(Ordering::Relaxed, Ordering::Relaxed, |next| {
            next.checked_add(1)
        })
        .map_err(|_| WorldError::WorldIdExhausted)
}

use crate::{
    aabb::Aabb,
    aerodynamics::{AerodynamicProperties, Atmosphere},
    cluster::{ClusterPart, RigidCluster, Weld},
    collider::Collider,
    collision::{
        CcdMotion, CcdTrajectory, CollisionConfig, CollisionError, CollisionEvent,
        CollisionInteraction, CollisionPair, CollisionSystem, ContactInfo, SolverBody,
        SolverManifold, SolverMaterial, StepContext, spatial_index::SpatialIndex,
    },
    containers::DensePool,
    execution::{SequentialExecutor, StepExecutor},
    gravity::{CentralGravity, Gravity, GravityError},
    joint::{Joint, JointBreakEvent, JointFrame},
    mass::MassProperties,
    material::{MaterialId, MaterialStorage, PhysicsMaterial},
    rigid_body::{BodyMotion, BodyState, BodyType, CcdMode, RigidBody, predict},
    transform::Transform,
};

pub use crate::cluster::balance::{BalanceError as WeldBalanceError, WeldReaction};

/// One overloaded weld in a diagnostic failure plan. The reaction is the
/// parent-on-child load at the child-side anchor from the sampled step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WeldFailureCandidate {
    pub reaction: WeldReaction,
    pub force_exceeded: bool,
    pub torque_exceeded: bool,
}

struct MeshAerodynamics {
    identity: Option<std::sync::Arc<()>>,
    cache: crate::aerodynamics::cache::PreparedGeometryCache,
    config: crate::aerodynamics::geometry::GeometryConfig,
    sound_speed: f64,
}

struct BodyEntry {
    body: RigidBody,
    first_collider: Option<ColliderId>,
    cluster: Option<RigidCluster>,
    mesh_aerodynamics: Option<MeshAerodynamics>,
    collider_aerodynamic_mesh: Option<crate::shape::TriangleMesh>,
    collider_tessellation: crate::aerodynamics::ColliderTessellation,
}

/// Object receiving a persistent force source.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ForceTarget {
    Body(BodyId),
    Part(PartId),
}

/// Persistent force applied at a point expressed in body-local coordinates.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ForceSource {
    body: BodyId,
    target: ForceTarget,
    local_point: Vec3,
    local_force: Vec3,
    enabled: bool,
}

/// One part-targeted persistent force source in the last successful full step.
/// Force is world-space N; moment is world-space N·m about the owning body's
/// center of mass at evaluation time. This is not a complete part or weld load.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PartForceSourceWrench {
    pub source: ForceSourceId,
    pub part: PartId,
    pub body: BodyId,
    pub force: Vec3,
    pub moment: Vec3,
}

/// Gravity contribution attributed to a cluster part for the last full step.
/// The acceleration is sampled at the cluster COM, as in the body model, and
/// treated as uniform across the cluster. Vectors are world-space, force in N
/// and moment in N·m about the cluster COM; gravity gradients are not modeled.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PartGravityWrench {
    pub body: BodyId,
    pub part: PartId,
    pub force: Vec3,
    pub moment: Vec3,
}

/// A single part-owned hull section's aerodynamic contribution. The force is
/// in world-space N and moment in world-space N·m about the body COM at
/// evaluation time; this is not a complete part load or a weld reaction.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PartHullWrench {
    pub body: BodyId,
    pub part: PartId,
    pub wrench: crate::aerodynamics::pressure::PressureWrench,
}

/// One part-owned wing's world-space force (N) and moment (N·m) about
/// the body COM at evaluation time. Not a complete part or weld load.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PartWingWrench {
    pub body: BodyId,
    pub part: PartId,
    pub wrench: crate::aerodynamics::pressure::PressureWrench,
}

/// A recorded velocity-contact impulse. The impulse acts on B; A receives its
/// negative. Points are world-space metres at velocity-solve time, impulse is
/// in N·s. Static and kinematic sides do not receive a velocity change.
/// This diagnostic is not a complete part-load ledger or weld reaction.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContactImpulse {
    pub pair: crate::collision::CollisionPair,
    pub parts: [Option<PartId>; 2],
    pub points: [Vec3; 2],
    /// Poses at the velocity solve, before position projection, when available.
    /// Presence does not certify a complete impulse ledger or event timing.
    pub velocity_transforms: Option<[Transform; 2]>,
    /// Before and after solver states for an isolated discrete contact when
    /// available. The outer array is before/after, the inner array is A/B.
    /// Other load paths must still be checked before weld-balance use.
    pub isolated_velocity_states: Option<[[BodyState; 2]; 2]>,
    pub impulse_on_b: Vec3,
    pub ccd: bool,
}

/// State of a cluster body at the beginning of the last successful full step.
/// This is the evaluation-time state for the corresponding force snapshots;
/// it is not a certificate that all loads are attributed to parts.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ClusterLoadState {
    /// Cluster body identified at force-evaluation time.
    pub body: BodyId,
    /// Root part at force-evaluation time.
    pub root: PartId,
    /// World-space pose and velocities before integration.
    pub state: BodyState,
    pub(crate) mass: Option<MassProperties>,
    // Continuous-load certificates at force evaluation, independent of collisions.
    // They do not certify that contact or CCD impulses have been accounted for.
    pub(crate) force_free_certified: bool,
    pub(crate) part_sources_certified: bool,
    pub(crate) part_hull_certified: bool,
    pub(crate) part_wings_certified: bool,
    pub(crate) part_surfaces_certified: bool,
    pub(crate) collision_loads_uncertified: bool,
    pub(crate) part_surface_count: usize,
    pub(crate) vacuum: bool,
    pub(crate) part_geometry: bool,
}

impl ClusterLoadState {
    fn invalidate_collision_loads(&mut self) {
        self.collision_loads_uncertified = true;
    }
}

#[derive(Clone, Copy)]
enum WeldLoadMode {
    Sources,
    Hull,
    PartAerodynamics,
}

/// Part mass and body-local pose at the same time as the load snapshots.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ClusterLoadPart {
    pub part: PartId,
    pub body: BodyId,
    pub local_transform: Transform,
    pub mass: MassProperties,
}

/// Weld topology and properties at the same time as the load snapshots.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ClusterLoadWeld {
    pub weld: WeldId,
    pub value: Weld,
}

/// Body-level simple-surface loads before and after the shared timestep
/// corrections. World-space N and N·m about the body COM at evaluation time.
/// These sums include both body- and part-targeted surfaces; their difference
/// is not attributed to individual parts and must not be treated as a weld load.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SimpleSurfaceCorrection {
    pub body: BodyId,
    pub raw_drag: crate::aerodynamics::pressure::PressureWrench,
    pub raw_lift: crate::aerodynamics::pressure::PressureWrench,
    pub applied_drag: crate::aerodynamics::pressure::PressureWrench,
    pub applied_lift: crate::aerodynamics::pressure::PressureWrench,
}

/// Uncorrected simple-surface force and torque, in world N and N·m about
/// the owning body's evaluation-time COM. This is not an applied part load:
/// aggregate timestep corrections are reported separately at body level.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RawSimpleSurfaceWrench {
    pub body: BodyId,
    pub target: AerodynamicTarget,
    pub drag: crate::aerodynamics::pressure::PressureWrench,
    pub lift: crate::aerodynamics::pressure::PressureWrench,
}

/// Why raw simple-surface loads cannot safely be used as applied part loads.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SimpleSurfaceAttributionError {
    /// The aggregate timestep correction changes drag or lift.
    CorrectedAggregate,
    /// At least one recorded surface targets the body rather than a part.
    BodyTarget,
    /// Raw entries are absent, incomplete, or inconsistent with the aggregate.
    InconsistentSnapshot,
}

impl SimpleSurfaceCorrection {
    /// Checks whether this body's raw simple-surface snapshot can be used as
    /// the applied per-part surface contributions, without allocating or
    /// inventing a distribution of the body-level correction. This says
    /// nothing about other forces, impulses, inertia, or weld equilibrium.
    /// Exact equality is intentional: no nonzero correction is discarded.
    pub fn check_part_attribution(
        &self,
        raw: &[RawSimpleSurfaceWrench],
    ) -> Result<(), SimpleSurfaceAttributionError> {
        if self.raw_drag != self.applied_drag || self.raw_lift != self.applied_lift {
            return Err(SimpleSurfaceAttributionError::CorrectedAggregate);
        }
        if raw
            .iter()
            .any(|entry| entry.body == self.body && entry.target == AerodynamicTarget::Body)
        {
            return Err(SimpleSurfaceAttributionError::BodyTarget);
        }
        let mut drag_force = Vec3::ZERO;
        let mut drag_moment = Vec3::ZERO;
        let mut lift_force = Vec3::ZERO;
        let mut lift_moment = Vec3::ZERO;
        let mut found = false;
        for entry in raw.iter().filter(|entry| entry.body == self.body) {
            found = true;
            drag_force = drag_force
                .checked_add(entry.drag.force)
                .ok_or(SimpleSurfaceAttributionError::InconsistentSnapshot)?;
            drag_moment = drag_moment
                .checked_add(entry.drag.moment)
                .ok_or(SimpleSurfaceAttributionError::InconsistentSnapshot)?;
            lift_force = lift_force
                .checked_add(entry.lift.force)
                .ok_or(SimpleSurfaceAttributionError::InconsistentSnapshot)?;
            lift_moment = lift_moment
                .checked_add(entry.lift.moment)
                .ok_or(SimpleSurfaceAttributionError::InconsistentSnapshot)?;
        }
        if !found
            || drag_force != self.raw_drag.force
            || drag_moment != self.raw_drag.moment
            || lift_force != self.raw_lift.force
            || lift_moment != self.raw_lift.moment
        {
            return Err(SimpleSurfaceAttributionError::InconsistentSnapshot);
        }
        Ok(())
    }
}

impl ForceSource {
    /// Returns the dynamic body receiving this force.
    pub fn body(&self) -> BodyId {
        self.body
    }

    /// Returns the body or cluster part receiving this force.
    pub fn target(&self) -> ForceTarget {
        self.target
    }

    /// Returns the application point relative to the body's origin, in meters.
    pub fn local_point(&self) -> Vec3 {
        self.local_point
    }

    /// Returns the force vector in body-local coordinates, in newtons.
    pub fn local_force(&self) -> Vec3 {
        self.local_force
    }

    /// Returns whether this source is applied on each physics step.
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }
}

struct ForceSourceBodyUpdate {
    id: BodyId,
    original: BodyMotion,
    prepared: BodyMotion,
}

struct JointEntry {
    target_a: JointTarget,
    target_b: JointTarget,
    body_a: BodyId,
    body_b: BodyId,
    joint: Joint,
}

/// Object used as one endpoint of a persistent joint.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JointTarget {
    Body(BodyId),
    Part(PartId),
}

struct GravityBodyUpdate {
    id: BodyId,
    error: Option<StepError>,
    original: BodyMotion,
    prepared: BodyMotion,
}

struct AerodynamicsBodyUpdate {
    id: BodyId,
    error: Option<StepError>,
    original: BodyMotion,
    prepared: BodyMotion,
    loads: PreparedAerodynamicLoads,
    part_hull_wrenches: Vec<PartHullWrench>,
    part_wing_wrenches: Vec<PartWingWrench>,
    simple_surface_correction: Option<SimpleSurfaceCorrection>,
    raw_simple_surfaces: Vec<RawSimpleSurfaceWrench>,
    applied_mesh_pressure: Option<crate::aerodynamics::pressure::PressureWrench>,
}

struct PreparedAerodynamicLoads {
    mesh_patches: Option<crate::aerodynamics::geometry::PreparedSurfacePatches>,
    sphere_patches: Option<Vec<crate::aerodynamics::pressure::PressurePatch>>,
    voxel_wave: Option<crate::aerodynamics::hull::HullSection>,
    assembly_wave: Option<(PartId, crate::aerodynamics::hull::HullSection)>,
    wing_visibility: Option<Vec<assembly_aerodynamics::PreparedWingVisibility>>,
}

/// Borrowed inputs prepared before aerodynamic loads are evaluated.
struct AerodynamicsInputs<'a> {
    mesh_sound_speed: f64,
    cluster: Option<&'a RigidCluster>,
    dt: Option<f64>,
    mesh_patches: Option<&'a crate::aerodynamics::geometry::PreparedSurfacePatches>,
    sphere_patches: Option<&'a [crate::aerodynamics::pressure::PressurePatch]>,
    voxel_wave: Option<&'a crate::aerodynamics::hull::HullSection>,
    assembly_wave: Option<&'a (PartId, crate::aerodynamics::hull::HullSection)>,
    wing_visibility: Option<&'a [assembly_aerodynamics::PreparedWingVisibility]>,
    base_pressure_patches: &'a [(
        crate::aerodynamics::pressure::PressurePatch,
        crate::aerodynamics::pressure::BasePressure,
    )],
    part_hull_wrenches: Option<&'a mut Vec<PartHullWrench>>,
    part_wing_wrenches: Option<&'a mut Vec<PartWingWrench>>,
    simple_surface_correction: Option<&'a mut Option<SimpleSurfaceCorrection>>,
    raw_simple_surfaces: Option<&'a mut Vec<RawSimpleSurfaceWrench>>,
    applied_mesh_pressure: Option<&'a mut Option<crate::aerodynamics::pressure::PressureWrench>>,
}

impl<'a> AerodynamicsInputs<'a> {
    fn new(mesh_sound_speed: f64) -> Self {
        Self {
            mesh_sound_speed,
            cluster: None,
            dt: None,
            mesh_patches: None,
            sphere_patches: None,
            voxel_wave: None,
            assembly_wave: None,
            wing_visibility: None,
            base_pressure_patches: &[],
            part_hull_wrenches: None,
            part_wing_wrenches: None,
            simple_surface_correction: None,
            raw_simple_surfaces: None,
            applied_mesh_pressure: None,
        }
    }

    fn with_cluster(mut self, cluster: Option<&'a RigidCluster>) -> Self {
        self.cluster = cluster;
        self
    }

    fn with_part_hull_wrenches(mut self, wrenches: &'a mut Vec<PartHullWrench>) -> Self {
        self.part_hull_wrenches = Some(wrenches);
        self
    }

    fn with_part_wing_wrenches(mut self, wrenches: &'a mut Vec<PartWingWrench>) -> Self {
        self.part_wing_wrenches = Some(wrenches);
        self
    }

    fn with_simple_surface_correction(
        mut self,
        correction: &'a mut Option<SimpleSurfaceCorrection>,
    ) -> Self {
        self.simple_surface_correction = Some(correction);
        self
    }

    fn with_raw_simple_surfaces(mut self, entries: &'a mut Vec<RawSimpleSurfaceWrench>) -> Self {
        self.raw_simple_surfaces = Some(entries);
        self
    }

    fn with_applied_mesh_pressure(
        mut self,
        wrench: &'a mut Option<crate::aerodynamics::pressure::PressureWrench>,
    ) -> Self {
        self.applied_mesh_pressure = Some(wrench);
        self
    }

    fn with_base_pressure(
        mut self,
        patches: &'a [(
            crate::aerodynamics::pressure::PressurePatch,
            crate::aerodynamics::pressure::BasePressure,
        )],
    ) -> Self {
        self.base_pressure_patches = patches;
        self
    }

    fn with_timestep(mut self, dt: Option<f64>) -> Self {
        self.dt = dt;
        self
    }

    fn with_loads(mut self, loads: &'a PreparedAerodynamicLoads) -> Self {
        self.mesh_patches = loads.mesh_patches.as_ref();
        self.sphere_patches = loads.sphere_patches.as_deref();
        self.voxel_wave = loads.voxel_wave.as_ref();
        self.assembly_wave = loads.assembly_wave.as_ref();
        self.wing_visibility = loads.wing_visibility.as_deref();
        self
    }

    fn with_assembly(
        mut self,
        assembly: Option<&'a assembly_aerodynamics::PreparedAssembly>,
    ) -> Self {
        if let Some(assembly) = assembly {
            self.mesh_patches = assembly.patches.as_ref();
            self.assembly_wave = assembly.wave.as_ref();
            self.wing_visibility = Some(&assembly.wing_visibility);
        }
        self
    }

    fn with_voxel_wave(
        mut self,
        section: Option<&'a crate::aerodynamics::hull::HullSection>,
    ) -> Self {
        self.voxel_wave = section;
        self
    }
}

const FORCE_SOURCE_BODY_LINEAR_SEARCH_LIMIT: usize = 128;

struct ColliderEntry {
    collider: Collider,
    body: BodyId,
    prev: Option<ColliderId>,
    next: Option<ColliderId>,
    chunk: Option<ChunkId>,
}

#[derive(Clone, Copy)]
struct CcdBodyReplaySnapshot {
    prediction: usize,
    ccd_state: BodyState,
    predicted_state: BodyState,
    elapsed: f64,
    impact_count: u32,
    clipped: bool,
    completed: bool,
}

fn swept_bounds_for_trajectory(
    collider: &Collider,
    trajectory: &CcdTrajectory,
    end: BodyState,
) -> Result<Option<crate::aabb::Aabb>, CollisionError> {
    let local_bounds = collider.local_bounds()?;
    let start_transform = trajectory.transform_at(0.0)?;
    let end_transform = end
        .transform()
        .compose(&trajectory.local_transform())
        .map_err(|_| CollisionError::ArithmeticOverflow)?;
    if trajectory.has_linear_collider_origin(end) {
        let motion = CcdMotion::linear(start_transform, end_transform)?;
        return crate::collision::swept_bounds_motion(local_bounds, &motion);
    }
    let (translation_bound, rotation_bound, _) = trajectory.path_bounds(end)?;
    let motion = CcdMotion::trajectory(
        start_transform,
        end_transform,
        translation_bound,
        false,
        rotation_bound,
        *trajectory,
    )?;
    crate::collision::swept_bounds_motion(local_bounds, &motion)
}

/// Controls whether explicitly supported approximations may be used during a step.
///
/// Unsupported aerodynamic regimes and critical errors remain errors under both
/// policies. Realtime can skip uncertifiable automatic weld checks as documented.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ErrorPolicy {
    /// Allow documented, safe fallback models when available.
    #[default]
    Realtime,
    /// Reject unsupported regimes without automatic approximations.
    Strict,
}

/// Degradations observed in the last successfully committed full step.
///
/// Counts are per affected body, not per CCD event. The CCD limit policy is
/// configured independently of [`ErrorPolicy`], including in strict mode.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct StepDiagnostics {
    /// Bodies whose motion was clipped by the collision subsystem after its
    /// configured CCD event limit was reached.
    pub collision_ccd_limit_clipped_bodies: usize,
    /// Breakable clusters whose automatic weld checks were skipped because the
    /// external load balance could not be certified.
    pub weld_incomplete_load_clusters: usize,
    /// Subset of skipped clusters affected by a contact, CCD clipping, or
    /// unresolved collision ownership. Other incomplete paths may also exist.
    pub weld_collision_load_clusters: usize,
}

impl StepDiagnostics {
    /// Whether any recorded approximation was applied.
    pub fn used_fallback(self) -> bool {
        self.collision_ccd_limit_clipped_bodies != 0 || self.weld_incomplete_load_clusters != 0
    }
}

/// Owns rigid bodies and colliders and assigns IDs scoped to this world instance.
pub struct PhysicsWorld {
    world_id: u64,
    error_policy: ErrorPolicy,
    step_diagnostics: StepDiagnostics,
    pending_step_diagnostics: StepDiagnostics,
    body_revision: u64,
    automatic_weld_failure: Option<(f64, f64)>,
    automatic_weld_impulse_tolerances: Option<(f64, f64)>,
    #[cfg(test)]
    weld_split_fail_after: Option<usize>,
    bodies: DensePool<BodyEntry>,
    parts: DensePool<ClusterPart>,
    part_aerodynamic_geometry: Option<HashMap<PartId, assembly_aerodynamics::PartGeometry>>,
    assembly_aerodynamics: Option<HashMap<BodyId, assembly_aerodynamics::AssemblyAerodynamics>>,
    welds: DensePool<Weld>,
    colliders: DensePool<ColliderEntry>,
    chunks: DensePool<chunk::ChunkEntry>,
    force_sources: DensePool<ForceSource>,
    joints: DensePool<JointEntry>,
    force_source_body_updates: Vec<ForceSourceBodyUpdate>,
    part_force_source_published: Vec<PartForceSourceWrench>,
    part_force_source_pending: Vec<PartForceSourceWrench>,
    force_source_body_indices: Option<HashMap<BodyId, usize>>,
    central_gravity: Option<CentralGravity>,
    gravity_fields: DensePool<Gravity>,
    gravity_body_updates: Vec<GravityBodyUpdate>,
    part_gravity_published: Vec<PartGravityWrench>,
    part_gravity_pending: Vec<PartGravityWrench>,
    atmosphere: Atmosphere,
    /// Sound speed used by the Newtonian mesh pressure model (m/s).
    /// The default is 340 m/s; callers should set this for their atmosphere.
    mesh_sound_speed: f64,
    aerodynamics_body_updates: Vec<AerodynamicsBodyUpdate>,
    part_hull_published: Vec<PartHullWrench>,
    part_hull_pending: Vec<PartHullWrench>,
    part_wing_published: Vec<PartWingWrench>,
    part_wing_pending: Vec<PartWingWrench>,
    simple_surface_published: Vec<SimpleSurfaceCorrection>,
    simple_surface_pending: Vec<SimpleSurfaceCorrection>,
    raw_simple_published: Vec<RawSimpleSurfaceWrench>,
    raw_simple_pending: Vec<RawSimpleSurfaceWrench>,
    raw_simple_buffers: Vec<Vec<RawSimpleSurfaceWrench>>,
    contact_impulses_published: Vec<ContactImpulse>,
    contact_impulses_pending: Vec<ContactImpulse>,
    contact_impulses_omitted_published: usize,
    contact_impulses_omitted_pending: usize,
    contact_ccd_clipped_published: bool,
    contact_ccd_clipped_pending: bool,
    cluster_load_states_published: Vec<ClusterLoadState>,
    cluster_load_states_pending: Vec<ClusterLoadState>,
    cluster_load_indices: Option<HashMap<BodyId, usize>>,
    cluster_load_parts_published: Vec<ClusterLoadPart>,
    cluster_load_parts_pending: Vec<ClusterLoadPart>,
    cluster_load_welds_published: Vec<ClusterLoadWeld>,
    cluster_load_welds_pending: Vec<ClusterLoadWeld>,
    part_hull_buffers: Vec<Vec<PartHullWrench>>,
    part_wing_buffers: Vec<Vec<PartWingWrench>>,
    owner_pressure: Option<HashMap<BodyId, crate::aerodynamics::pressure::OwnerPressureWorkspace>>,
    owner_pressure_owners: Vec<crate::aerodynamics::geometry::GeometryOwner>,
    materials: MaterialStorage,
    collision_system: CollisionSystem,
    joint_events: Vec<JointBreakEvent>,
    query_index: Option<SpatialIndex>,
    query_index_dirty: bool,
    query_proxies: Vec<(ColliderId, Aabb)>,
    unbounded_query_colliders: Vec<ColliderId>,
}

/// Opaque copied body data used to predict and atomically commit a world step.
#[doc(hidden)]
#[derive(Clone, Copy, Debug)]
pub struct BodyStep {
    body_id: BodyId,
    source: BodyMotion,
    body_revision: u64,
    predicted: Option<BodyState>,
    dt: Option<f64>,
    error: Option<StepError>,
    prepared: BodyMotion,
}

impl BodyStep {
    pub fn body_id(&self) -> BodyId {
        self.body_id
    }

    pub fn integrate(&mut self, dt: f64) -> Result<(), StepError> {
        self.predicted = None;
        self.dt = None;
        self.error = None;
        let predicted = match predict(&self.prepared, dt) {
            Ok(predicted) => predicted,
            Err(cause) => {
                let error = StepError::BodyIntegration {
                    body_id: self.body_id,
                    cause,
                };
                self.error = Some(error);
                return Err(error);
            }
        };
        self.predicted = Some(predicted);
        self.dt = Some(dt);
        Ok(())
    }
}

impl PhysicsWorld {
    /// Diagnostics of the last committed full step, or empty before one succeeds.
    /// Failed steps and low-level shard operations do not publish diagnostics.
    pub fn step_diagnostics(&self) -> StepDiagnostics {
        self.step_diagnostics
    }

    /// Returns the policy used for full physics steps.
    pub fn error_policy(&self) -> ErrorPolicy {
        self.error_policy
    }

    /// Selects the policy used for subsequent steps.
    pub fn set_error_policy(&mut self, policy: ErrorPolicy) {
        self.error_policy = policy;
    }

    /// Enable automatic weld failure for certified full steps using absolute
    /// equilibrium tolerances in N and N·m. `None` disables it (the default).
    /// Incomplete loads skip fracture with diagnostics in `Realtime`; `Strict`
    /// rejects them before commit. Other balance errors fail in both policies.
    pub fn set_automatic_weld_failure(
        &mut self,
        tolerances: Option<(f64, f64)>,
    ) -> Result<(), WeldBalanceError> {
        if tolerances.is_some_and(|(force, moment)| {
            !force.is_finite() || force < 0.0 || !moment.is_finite() || moment < 0.0
        }) {
            return Err(WeldBalanceError::InvalidTolerance);
        }
        self.automatic_weld_failure = tolerances;
        Ok(())
    }

    /// Configure absolute linear and angular impulse-balance tolerances in
    /// N·s and N·m·s respectively. `None` clears the configuration (default).
    ///
    /// This only stores tolerances for future automatic impact checks; it does
    /// not enable such checks or change the handling of contacts. Both values
    /// must be finite and nonnegative. An invalid update preserves the previous
    /// configuration.
    pub fn set_automatic_weld_impulse_tolerances(
        &mut self,
        tolerances: Option<(f64, f64)>,
    ) -> Result<(), WeldBalanceError> {
        if tolerances.is_some_and(|(linear, angular)| {
            !linear.is_finite() || linear < 0.0 || !angular.is_finite() || angular < 0.0
        }) {
            return Err(WeldBalanceError::InvalidTolerance);
        }
        self.automatic_weld_impulse_tolerances = tolerances;
        Ok(())
    }

    /// Returns the configured impulse-balance tolerances, if any.
    pub fn automatic_weld_impulse_tolerances(&self) -> Option<(f64, f64)> {
        self.automatic_weld_impulse_tolerances
    }
    /// Plans weld failures for the last certified part-source snapshot without
    /// changing the world. Both thresholds compare vector magnitudes; equality
    /// does not exceed a threshold. This historical plan is not authorization
    /// to break welds after subsequent edits or integration.
    pub fn part_source_weld_failure_candidates(
        &self,
        body: BodyId,
        force_tolerance: f64,
        moment_tolerance: f64,
    ) -> Result<Vec<WeldFailureCandidate>, WeldBalanceError> {
        let reactions = self.part_source_weld_reactions(body, force_tolerance, moment_tolerance)?;
        Self::weld_failure_candidates_from_reactions(reactions, &self.cluster_load_welds_published)
    }

    fn weld_failure_candidates_from_reactions(
        reactions: Vec<WeldReaction>,
        snapshot_welds: &[ClusterLoadWeld],
    ) -> Result<Vec<WeldFailureCandidate>, WeldBalanceError> {
        use crate::cluster::WeldStrength;
        let mut welds = HashMap::new();
        welds
            .try_reserve(snapshot_welds.len())
            .map_err(|_| WeldBalanceError::Physics(ClusterError::StorageCapacityExceeded))?;
        for weld in snapshot_welds {
            if welds.insert(weld.weld, weld.value).is_some() {
                return Err(WeldBalanceError::InvalidTree);
            }
        }
        let mut candidates = Vec::new();
        candidates
            .try_reserve_exact(reactions.len())
            .map_err(|_| WeldBalanceError::Physics(ClusterError::StorageCapacityExceeded))?;
        for reaction in reactions {
            let weld = welds
                .get(&reaction.weld)
                .ok_or(WeldBalanceError::InvalidTree)?;
            let WeldStrength::Breakable {
                max_force,
                max_torque,
            } = weld.desc().strength
            else {
                continue;
            };
            let force = reaction.force.length();
            let torque = reaction.moment.length();
            if !force.is_finite() || !torque.is_finite() {
                return Err(WeldBalanceError::Physics(ClusterError::ArithmeticOverflow));
            }
            let force_exceeded = max_force.is_some_and(|limit| force > limit);
            let torque_exceeded = max_torque.is_some_and(|limit| torque > limit);
            if force_exceeded || torque_exceeded {
                candidates.push(WeldFailureCandidate {
                    reaction,
                    force_exceeded,
                    torque_exceeded,
                });
            }
        }
        Ok(candidates)
    }

    /// Evaluate only data staged for the active full step, never a previously
    /// published diagnostic. Call after persistent sources have been prepared.
    #[allow(dead_code)]
    fn pending_weld_failure_candidates(
        &self,
        body: BodyId,
        force_tolerance: f64,
        moment_tolerance: f64,
    ) -> Result<Vec<WeldFailureCandidate>, WeldBalanceError> {
        let reactions = Self::part_source_weld_reactions_from_snapshot(
            body,
            force_tolerance,
            moment_tolerance,
            &self.cluster_load_states_pending,
            &self.cluster_load_parts_pending,
            &self.cluster_load_welds_pending,
            &self.part_force_source_pending,
            &self.part_gravity_pending,
            &self.part_hull_pending,
            &self.part_wing_pending,
            (&self.simple_surface_pending, &self.raw_simple_pending),
            self.owner_pressure
                .as_ref()
                .and_then(|map| map.get(&body))
                .and_then(|workspace| workspace.staged()),
            WeldLoadMode::PartAerodynamics,
        )?;
        Self::weld_failure_candidates_from_reactions(reactions, &self.cluster_load_welds_pending)
    }

    fn stage_automatic_weld_splits(
        &mut self,
    ) -> Result<Option<cluster::InsertedWeldSplits>, StepError> {
        let Some((force, moment)) = self.automatic_weld_failure else {
            return Ok(None);
        };
        let plans =
            self.prepare_pending_weld_splits(force, moment)
                .map_err(|error| match error {
                    cluster::WeldSplitPreparationError::Balance(cause) => {
                        StepError::WeldBalance(cause)
                    }
                    cluster::WeldSplitPreparationError::Cluster(cause) => {
                        StepError::WeldSplit(cause)
                    }
                })?;
        if plans.is_empty() {
            return Ok(None);
        }
        self.insert_weld_split_bodies(plans)
            .map(Some)
            .map_err(StepError::WeldSplit)
    }
    /// Calculates instantaneous parent-on-child weld reactions from the last
    /// successful full-step snapshot. Currently only force-free
    /// clusters are supported. This diagnostic does not break welds. Tolerances
    /// are absolute, in N and N·m. Topology and loads come from the same step;
    /// later world edits do not alter the snapshot.
    pub fn force_free_weld_reactions(
        &self,
        body: BodyId,
        force_tolerance: f64,
        moment_tolerance: f64,
    ) -> Result<Vec<WeldReaction>, WeldBalanceError> {
        let state = self
            .cluster_load_states_published
            .iter()
            .find(|state| state.body == body)
            .ok_or(WeldBalanceError::IncompleteLoads)?;
        if !state.force_free_certified || state.collision_loads_uncertified {
            return Err(WeldBalanceError::IncompleteLoads);
        }
        self.part_source_weld_reactions(body, force_tolerance, moment_tolerance)
    }

    /// Instantaneous reactions for clusters subject only to
    /// part-targeted persistent forces and gravity sampled at the cluster COM.
    /// The cluster must have no colliders; the world must have no joints or
    /// chunks, and the cluster must have no aerodynamic loads.
    /// Results are diagnostics from the last successful full-step snapshot.
    pub fn part_source_weld_reactions(
        &self,
        body: BodyId,
        force_tolerance: f64,
        moment_tolerance: f64,
    ) -> Result<Vec<WeldReaction>, WeldBalanceError> {
        Self::part_source_weld_reactions_from_snapshot(
            body,
            force_tolerance,
            moment_tolerance,
            &self.cluster_load_states_published,
            &self.cluster_load_parts_published,
            &self.cluster_load_welds_published,
            &self.part_force_source_published,
            &self.part_gravity_published,
            &[],
            &[],
            (&[], &[]),
            None,
            WeldLoadMode::Sources,
        )
    }

    /// Reactions including part-owned hull sections from the last full step.
    /// Other aerodynamic models, surfaces, wings, contacts and joints remain
    /// unsupported. Gravity uses the same COM sampling as the body model.
    pub fn part_hull_weld_reactions(
        &self,
        body: BodyId,
        force_tolerance: f64,
        moment_tolerance: f64,
    ) -> Result<Vec<WeldReaction>, WeldBalanceError> {
        Self::part_source_weld_reactions_from_snapshot(
            body,
            force_tolerance,
            moment_tolerance,
            &self.cluster_load_states_published,
            &self.cluster_load_parts_published,
            &self.cluster_load_welds_published,
            &self.part_force_source_published,
            &self.part_gravity_published,
            &self.part_hull_published,
            &[],
            (&[], &[]),
            None,
            WeldLoadMode::Hull,
        )
    }

    /// Reactions including part-owned hull and wing loads from the same step.
    /// Body-owned aerodynamics, common surface corrections, non-Mesh geometry
    /// and contacts are not certified. Part-owned Mesh pressure is supported
    /// only when its same-step owner snapshot matches the applied wrench.
    pub fn part_aerodynamic_weld_reactions(
        &self,
        body: BodyId,
        force_tolerance: f64,
        moment_tolerance: f64,
    ) -> Result<Vec<WeldReaction>, WeldBalanceError> {
        Self::part_source_weld_reactions_from_snapshot(
            body,
            force_tolerance,
            moment_tolerance,
            &self.cluster_load_states_published,
            &self.cluster_load_parts_published,
            &self.cluster_load_welds_published,
            &self.part_force_source_published,
            &self.part_gravity_published,
            &self.part_hull_published,
            &self.part_wing_published,
            (&self.simple_surface_published, &self.raw_simple_published),
            self.owner_pressure
                .as_ref()
                .and_then(|map| map.get(&body))
                .map(|workspace| workspace.published()),
            WeldLoadMode::PartAerodynamics,
        )
    }

    /// All slices must belong to the same evaluation-time transaction.
    fn part_source_weld_reactions_from_snapshot(
        body: BodyId,
        force_tolerance: f64,
        moment_tolerance: f64,
        states: &[ClusterLoadState],
        parts: &[ClusterLoadPart],
        snapshot_welds: &[ClusterLoadWeld],
        sources: &[PartForceSourceWrench],
        gravity: &[PartGravityWrench],
        hull: &[PartHullWrench],
        wings: &[PartWingWrench],
        simple: (&[SimpleSurfaceCorrection], &[RawSimpleSurfaceWrench]),
        pressure: Option<
            &[(
                crate::aerodynamics::geometry::GeometryOwner,
                crate::aerodynamics::pressure::PressureWrench,
            )],
        >,
        load_mode: WeldLoadMode,
    ) -> Result<Vec<WeldReaction>, WeldBalanceError> {
        use crate::cluster::balance::{
            RecordedPartLoads, prepare_recorded_loads, prepare_tree, reaction_moment_at_anchor,
            tree_reactions,
        };
        let state = states
            .iter()
            .find(|state| state.body == body)
            .ok_or(WeldBalanceError::IncompleteLoads)?;
        if state.collision_loads_uncertified
            || !match load_mode {
                WeldLoadMode::Sources => state.part_sources_certified,
                WeldLoadMode::Hull => state.part_hull_certified,
                WeldLoadMode::PartAerodynamics => state.part_surfaces_certified,
            }
        {
            return Err(WeldBalanceError::IncompleteLoads);
        }
        let tree = prepare_tree(state, parts, snapshot_welds)?;
        let mass = state.mass.ok_or(WeldBalanceError::IncompleteLoads)?;
        let (center, inertia) =
            crate::cluster::motion::world_properties(mass, state.state.transform())
                .map_err(WeldBalanceError::Physics)?;
        let origin = center;
        let mut inputs = prepare_recorded_loads(
            state,
            &tree,
            RecordedPartLoads {
                sources,
                gravity,
                hull,
                wings,
            },
            center,
            origin,
        )?;
        if state.part_geometry && !state.vacuum {
            use crate::aerodynamics::geometry::GeometryOwner;
            let pressure = pressure.ok_or(WeldBalanceError::IncompleteLoads)?;
            let mut indices = HashMap::new();
            indices
                .try_reserve(tree.len())
                .map_err(|_| WeldBalanceError::Physics(ClusterError::StorageCapacityExceeded))?;
            for (index, part) in tree.iter().enumerate() {
                indices.insert(part.part, index);
            }
            for (owner, wrench) in pressure {
                let GeometryOwner::Part(part) = owner else {
                    return Err(WeldBalanceError::IncompleteLoads);
                };
                let index = *indices.get(part).ok_or(WeldBalanceError::InvalidTree)?;
                let input = &mut inputs[index];
                input.external_force = input
                    .external_force
                    .checked_add(wrench.force)
                    .ok_or(WeldBalanceError::Physics(ClusterError::ArithmeticOverflow))?;
                input.external_moment = input
                    .external_moment
                    .checked_add(wrench.moment)
                    .ok_or(WeldBalanceError::Physics(ClusterError::ArithmeticOverflow))?;
            }
        }
        if matches!(load_mode, WeldLoadMode::PartAerodynamics)
            && state.part_surface_count > 0
            && !state.vacuum
        {
            let (corrections, raw) = simple;
            let mut matching = corrections.iter().filter(|item| item.body == body);
            let correction = matching.next().ok_or(WeldBalanceError::IncompleteLoads)?;
            if matching.next().is_some()
                || raw.iter().filter(|item| item.body == body).count() != state.part_surface_count
                || correction.check_part_attribution(raw).is_err()
            {
                return Err(WeldBalanceError::IncompleteLoads);
            }
            let mut indices = HashMap::new();
            indices
                .try_reserve(tree.len())
                .map_err(|_| WeldBalanceError::Physics(ClusterError::StorageCapacityExceeded))?;
            for (index, part) in tree.iter().enumerate() {
                indices.insert(part.part, index);
            }
            for load in raw.iter().filter(|item| item.body == body) {
                let AerodynamicTarget::Part(part) = load.target else {
                    return Err(WeldBalanceError::IncompleteLoads);
                };
                let input =
                    &mut inputs[*indices.get(&part).ok_or(WeldBalanceError::InvalidTree)?];
                for wrench in [load.drag, load.lift] {
                    input.external_force = input
                        .external_force
                        .checked_add(wrench.force)
                        .ok_or(WeldBalanceError::Physics(ClusterError::ArithmeticOverflow))?;
                    input.external_moment = input
                        .external_moment
                        .checked_add(wrench.moment)
                        .ok_or(WeldBalanceError::Physics(ClusterError::ArithmeticOverflow))?;
                }
            }
        }
        let mut force = Vec3::ZERO;
        let mut moment = Vec3::ZERO;
        for input in &mut inputs {
            force = force
                .checked_add(input.external_force)
                .ok_or(WeldBalanceError::Physics(ClusterError::ArithmeticOverflow))?;
            moment = moment
                .checked_add(input.external_moment)
                .ok_or(WeldBalanceError::Physics(ClusterError::ArithmeticOverflow))?;
            input.loads_complete = true;
        }
        let acceleration = force
            .checked_mul_scalar(1.0 / mass.mass())
            .ok_or(WeldBalanceError::Physics(ClusterError::ArithmeticOverflow))?;
        let omega = state.state.angular_velocity();
        let effective_moment = inertia
            .checked_mul_vector(omega)
            .and_then(|angular_momentum| omega.checked_cross(angular_momentum))
            .and_then(|gyroscopic| moment.checked_sub(gyroscopic))
            .ok_or(WeldBalanceError::Physics(ClusterError::ArithmeticOverflow))?;
        let alpha = inertia
            .solve_spd(effective_moment)
            .map_err(|_| WeldBalanceError::Physics(ClusterError::InvalidMotion))?;
        let mut parents = Vec::new();
        parents
            .try_reserve_exact(tree.len())
            .map_err(|_| WeldBalanceError::Physics(ClusterError::StorageCapacityExceeded))?;
        parents.extend(tree.iter().map(|part| part.parent));
        let reactions = tree_reactions(
            &inputs,
            &parents,
            center,
            acceleration,
            omega,
            alpha,
            origin,
            force_tolerance,
            moment_tolerance,
        )?;
        let mut result = Vec::new();
        result
            .try_reserve_exact(tree.len().saturating_sub(1))
            .map_err(|_| WeldBalanceError::Physics(ClusterError::StorageCapacityExceeded))?;
        let mut welds = HashMap::new();
        welds
            .try_reserve(snapshot_welds.len())
            .map_err(|_| WeldBalanceError::Physics(ClusterError::StorageCapacityExceeded))?;
        for weld in snapshot_welds {
            if welds.insert(weld.weld, weld.value).is_some() {
                return Err(WeldBalanceError::InvalidTree);
            }
        }
        for (index, part) in tree.iter().enumerate().skip(1) {
            let weld_id = part.weld.ok_or(WeldBalanceError::InvalidTree)?;
            let weld = welds.get(&weld_id).ok_or(WeldBalanceError::InvalidTree)?;
            let frame = if weld.part_a() == part.part {
                weld.desc().frame_a
            } else if weld.part_b() == part.part {
                weld.desc().frame_b
            } else {
                return Err(WeldBalanceError::InvalidTree);
            };
            let anchor = part.pose.transform_point(frame.local_anchor);
            let (force, moment) = reactions[index];
            let moment = reaction_moment_at_anchor(force, moment, origin, anchor)?;
            result.push(WeldReaction {
                weld: weld_id,
                child: part.part,
                force,
                moment,
                anchor,
            });
        }
        Ok(result)
    }
    /// Cluster-body states from the beginning of the last successful full step.
    /// Low-level shard commits do not publish this diagnostic. Later topology
    /// edits can make its body IDs or current part transforms stale.
    pub fn cluster_load_states(&self) -> &[ClusterLoadState] {
        &self.cluster_load_states_published
    }

    /// Part mass and local pose from the last successful full step.
    pub fn cluster_load_parts(&self) -> &[ClusterLoadPart] {
        &self.cluster_load_parts_published
    }

    /// Welds from the last successful full step, independent of later edits.
    pub fn cluster_load_welds(&self) -> &[ClusterLoadWeld] {
        &self.cluster_load_welds_published
    }

    /// Recorded velocity-contact impulses from the last successful full step.
    /// This is diagnostic data, not a complete contact or weld-load balance.
    /// Low-level shard commits do not publish this snapshot.
    pub fn contact_impulses(&self) -> &[ContactImpulse] {
        &self.contact_impulses_published
    }

    /// Number of staged contact points missing velocity-solve data in the
    /// last successful full step. Zero does not establish completeness of
    /// other solver paths, CCD replay, or the overall part-load balance.
    pub fn contact_impulses_omitted_points(&self) -> usize {
        self.contact_impulses_omitted_published
    }

    /// Whether CCD clipped any dynamic body during the last successful full
    /// step. Clipping changes velocity without a recorded physical impulse;
    /// contact snapshots from such a step cannot form a complete load balance.
    /// False does not prove that other contact paths are complete.
    pub fn contact_impulses_ccd_clipped(&self) -> bool {
        self.contact_ccd_clipped_published
    }

    /// Part-targeted persistent-source contributions from the last successful
    /// full step. Entries preserve source iteration order and are not summed.
    /// They are a historical snapshot, including IDs of subsequently removed
    /// parts or sources; low-level prediction APIs do not update the snapshot.
    pub fn part_force_source_wrenches(&self) -> &[PartForceSourceWrench] {
        &self.part_force_source_published
    }

    /// Per-part gravity attribution from the last successful full step. This
    /// historical snapshot is not a weld reaction or a gravity-gradient model.
    pub fn part_gravity_wrenches(&self) -> &[PartGravityWrench] {
        &self.part_gravity_published
    }

    /// Part hull contributions from the last successful full step. This is a
    /// historical snapshot and excludes wings, mesh pressure and other loads.
    pub fn part_hull_wrenches(&self) -> &[PartHullWrench] {
        &self.part_hull_published
    }

    /// Part wing contributions from the last successful full step, excluding
    /// hulls, mesh pressure and all non-aerodynamic loads.
    pub fn part_wing_wrenches(&self) -> &[PartWingWrench] {
        &self.part_wing_published
    }

    /// Aggregate simple-surface corrections from the last successful full step.
    pub fn simple_surface_corrections(&self) -> &[SimpleSurfaceCorrection] {
        &self.simple_surface_published
    }

    /// Raw simple-surface wrenches from the last successful full step. Body
    /// and part targets are included; zero-density surfaces are omitted.
    /// These are before the shared correction and are not applied part loads.
    pub fn raw_simple_surface_wrenches(&self) -> &[RawSimpleSurfaceWrench] {
        &self.raw_simple_published
    }

    /// Impact-pressure contribution from prepared mesh patches in the last
    /// successful full step. Forces are world-space newtons; moments are world
    /// N·m about the body's center of mass at evaluation time. This excludes
    /// other aerodynamic and mechanical loads and must not be used as a weld
    /// reaction. Low-level prediction/commit APIs do not publish this data.
    /// This diagnostic currently repeats mesh-pressure evaluation and uses
    /// reusable workspace storage. No result is exposed for a deleted body,
    /// vacuum, zero density, or a step without prepared mesh pressure.
    pub fn owner_mesh_pressure(
        &self,
        body: BodyId,
    ) -> Option<
        &[(
            crate::aerodynamics::geometry::GeometryOwner,
            crate::aerodynamics::pressure::PressureWrench,
        )],
    > {
        if body.world != self.world_id || self.bodies.get(body.dense).is_none() {
            return None;
        }
        self.owner_pressure
            .as_ref()?
            .get(&body)
            .map(|workspace| workspace.published())
            .filter(|loads| !loads.is_empty())
    }

    pub const fn new() -> Self {
        Self {
            world_id: 0,
            error_policy: ErrorPolicy::Realtime,
            step_diagnostics: StepDiagnostics {
                collision_ccd_limit_clipped_bodies: 0,
                weld_incomplete_load_clusters: 0,
                weld_collision_load_clusters: 0,
            },
            pending_step_diagnostics: StepDiagnostics {
                collision_ccd_limit_clipped_bodies: 0,
                weld_incomplete_load_clusters: 0,
                weld_collision_load_clusters: 0,
            },
            body_revision: 0,
            automatic_weld_failure: None,
            automatic_weld_impulse_tolerances: None,
            #[cfg(test)]
            weld_split_fail_after: None,
            bodies: DensePool::new(),
            parts: DensePool::new(),
            part_aerodynamic_geometry: None,
            assembly_aerodynamics: None,
            welds: DensePool::new(),
            colliders: DensePool::new(),
            chunks: DensePool::new(),
            force_sources: DensePool::new(),
            joints: DensePool::new(),
            force_source_body_updates: Vec::new(),
            part_force_source_published: Vec::new(),
            part_force_source_pending: Vec::new(),
            force_source_body_indices: None,
            central_gravity: None,
            gravity_fields: DensePool::new(),
            gravity_body_updates: Vec::new(),
            part_gravity_published: Vec::new(),
            part_gravity_pending: Vec::new(),
            atmosphere: Atmosphere::Vacuum,
            mesh_sound_speed: 340.0,
            aerodynamics_body_updates: Vec::new(),
            part_hull_published: Vec::new(),
            part_hull_pending: Vec::new(),
            part_wing_published: Vec::new(),
            part_wing_pending: Vec::new(),
            simple_surface_published: Vec::new(),
            simple_surface_pending: Vec::new(),
            raw_simple_published: Vec::new(),
            raw_simple_pending: Vec::new(),
            raw_simple_buffers: Vec::new(),
            contact_impulses_published: Vec::new(),
            contact_impulses_pending: Vec::new(),
            contact_impulses_omitted_published: 0,
            contact_impulses_omitted_pending: 0,
            contact_ccd_clipped_published: false,
            contact_ccd_clipped_pending: false,
            cluster_load_states_published: Vec::new(),
            cluster_load_states_pending: Vec::new(),
            cluster_load_indices: None,
            cluster_load_parts_published: Vec::new(),
            cluster_load_parts_pending: Vec::new(),
            cluster_load_welds_published: Vec::new(),
            cluster_load_welds_pending: Vec::new(),
            part_hull_buffers: Vec::new(),
            part_wing_buffers: Vec::new(),
            owner_pressure: None,
            owner_pressure_owners: Vec::new(),
            materials: MaterialStorage::new(),
            collision_system: CollisionSystem::new(),
            joint_events: Vec::new(),
            query_index: None,
            query_index_dirty: false,
            query_proxies: Vec::new(),
            unbounded_query_colliders: Vec::new(),
        }
    }

    pub fn collision_config(&self) -> CollisionConfig {
        self.collision_system.config()
    }

    pub fn central_gravity(&self) -> Option<CentralGravity> {
        self.central_gravity
    }

    pub fn set_central_gravity(&mut self, gravity: Option<CentralGravity>) {
        self.central_gravity = gravity;
    }

    pub fn atmosphere(&self) -> Atmosphere {
        self.atmosphere
    }

    pub fn set_atmosphere(&mut self, atmosphere: Atmosphere) {
        self.atmosphere = atmosphere;
    }

    /// Sets the uniform sound speed used by Mesh/Colliders, in m/s.
    /// Must be finite and positive and match the configured atmospheric medium.
    /// The compatibility default is 340 m/s; density alone cannot determine it.
    /// This changes Mach classification without invalidating local geometry.
    pub fn set_mesh_sound_speed(
        &mut self,
        sound_speed: f64,
    ) -> Result<(), crate::aerodynamics::pressure::PressureError> {
        if !sound_speed.is_finite() || sound_speed <= 0.0 {
            return Err(crate::aerodynamics::pressure::PressureError::InvalidSoundSpeed);
        }
        self.mesh_sound_speed = sound_speed;
        Ok(())
    }

    /// Returns the uniform Mesh/Colliders sound speed in m/s.
    pub fn mesh_sound_speed(&self) -> f64 {
        self.mesh_sound_speed
    }

    /// Selects triangles from one attached collider's current tessellation.
    /// Indices use `collider_mesh([collider], body's tessellation)` ordering.
    /// The caller must establish outward winding, full exposure and base/wake
    /// applicability; no overlap or shielding check is performed. Replacing or
    /// mutating colliders or tessellation clears the resulting configuration.
    /// Errors leave the previous selection unchanged.
    pub fn set_collider_base_triangles(
        &mut self,
        body: BodyId,
        collider: ColliderId,
        indices: &[usize],
        base: crate::aerodynamics::pressure::BasePressure,
    ) -> Result<(), crate::aerodynamics::AerodynamicsError> {
        use crate::aerodynamics::{Aerodynamics, AerodynamicsError};
        if body.world != self.world_id || collider.world != self.world_id {
            return Err(AerodynamicsError::UnsupportedModel);
        }
        let entry = self
            .bodies
            .get(body.dense)
            .ok_or(AerodynamicsError::UnsupportedModel)?;
        if entry.cluster.is_some() || !matches!(entry.body.aerodynamics(), Aerodynamics::Colliders)
        {
            return Err(AerodynamicsError::UnsupportedModel);
        }
        let source = self
            .colliders
            .get(collider.dense)
            .filter(|entry| entry.body == body && entry.chunk.is_none())
            .ok_or(AerodynamicsError::UnsupportedModel)?;
        let mesh = crate::aerodynamics::colliders::collider_mesh(
            [&source.collider],
            entry.collider_tessellation,
        )?;
        let patches = RigidBody::base_triangles_from_mesh(&mesh, indices, base)
            .map_err(AerodynamicsError::Pressure)?;
        let triangles = indices
            .iter()
            .map(|&i| mesh.triangles()[i].map(|v| mesh.vertices()[v]))
            .collect();
        let body = &mut self
            .bodies
            .get_mut(body.dense)
            .ok_or(AerodynamicsError::UnsupportedModel)?
            .body;
        body.set_base_pressure_patches(patches);
        body.base_pressure_triangles = Some(triangles);
        self.body_revision = self.body_revision.wrapping_add(1);
        Ok(())
    }

    pub fn aerodynamics(&self, id: BodyId) -> Option<&crate::aerodynamics::Aerodynamics> {
        self.body(id).map(RigidBody::aerodynamics)
    }

    /// Replaces the primary aerodynamic model, retaining additional surfaces.
    pub fn set_aerodynamics(
        &mut self,
        id: BodyId,
        aerodynamics: crate::aerodynamics::Aerodynamics,
    ) -> Result<(), WorldError> {
        if id.world != self.world_id {
            return Err(WorldError::InvalidBodyId);
        }
        if self.cluster(id).is_some()
            && matches!(
                aerodynamics,
                crate::aerodynamics::Aerodynamics::Mesh(_)
                    | crate::aerodynamics::Aerodynamics::VoxelWave(_)
                    | crate::aerodynamics::Aerodynamics::Colliders
            )
        {
            return Err(WorldError::UnsupportedAerodynamics);
        }
        let entry = self
            .bodies
            .get_mut(id.dense)
            .ok_or(WorldError::InvalidBodyId)?;
        entry.body.set_aerodynamics(aerodynamics);
        entry.mesh_aerodynamics = None;
        entry.collider_aerodynamic_mesh = None;
        self.body_revision = self.body_revision.wrapping_add(1);
        Ok(())
    }

    pub fn aerodynamic_surfaces(&self, id: BodyId) -> Option<&[AerodynamicSurface]> {
        if id.world != self.world_id {
            return None;
        }
        self.bodies
            .get(id.dense)
            .map(|entry| entry.body.aerodynamic_surfaces.as_slice())
    }

    /// Returns an index valid until the surface list changes or the cluster is split or merged.
    pub fn add_aerodynamic_surface(
        &mut self,
        id: BodyId,
        properties: AerodynamicProperties,
    ) -> Result<usize, WorldError> {
        if id.world != self.world_id {
            return Err(WorldError::InvalidBodyId);
        }
        let surfaces = &mut self
            .bodies
            .get_mut(id.dense)
            .ok_or(WorldError::InvalidBodyId)?
            .body
            .aerodynamic_surfaces;
        surfaces
            .try_reserve(1)
            .map_err(|_| WorldError::DensePoolCapacityExceeded)?;
        surfaces.push(AerodynamicSurface::new(properties));
        self.body_revision = self.body_revision.wrapping_add(1);
        Ok(surfaces.len() - 1)
    }

    /// Removes a surface by its current index, invalidating previously returned indices.
    pub fn remove_aerodynamic_surface(
        &mut self,
        id: BodyId,
        index: usize,
    ) -> Result<AerodynamicProperties, WorldError> {
        if id.world != self.world_id {
            return Err(WorldError::InvalidBodyId);
        }
        let surfaces = &mut self
            .bodies
            .get_mut(id.dense)
            .ok_or(WorldError::InvalidBodyId)?
            .body
            .aerodynamic_surfaces;
        if index >= surfaces.len() {
            return Err(WorldError::InvalidAerodynamicSurface);
        }
        let properties = surfaces.remove(index).properties();
        self.body_revision = self.body_revision.wrapping_add(1);
        Ok(properties)
    }

    /// Returns an index valid until the surface list changes or the cluster is split or merged.
    pub fn add_aerodynamic_surface_for_part(
        &mut self,
        part: PartId,
        properties: AerodynamicProperties,
    ) -> Result<usize, WorldError> {
        let part_entry = self.cluster_part(part).ok_or(WorldError::InvalidBodyId)?;
        let body = part_entry.body();
        let surfaces = &mut self
            .bodies
            .get_mut(body.dense)
            .ok_or(WorldError::InvalidBodyId)?
            .body
            .aerodynamic_surfaces;
        surfaces
            .try_reserve(1)
            .map_err(|_| WorldError::DensePoolCapacityExceeded)?;
        surfaces.push(AerodynamicSurface::for_part(part, properties));
        self.body_revision = self.body_revision.wrapping_add(1);
        Ok(surfaces.len() - 1)
    }

    pub(crate) fn rebase_aerodynamic_surface(
        surface: AerodynamicSurface,
        old_body: &BodyMotion,
        new_body: &BodyMotion,
    ) -> Result<AerodynamicSurface, crate::aerodynamics::AerodynamicsError> {
        if !matches!(surface.target(), AerodynamicTarget::Body) {
            return Ok(surface);
        }
        let old_center = old_body
            .world_center_of_mass()
            .map_err(|_| crate::aerodynamics::AerodynamicsError::ArithmeticOverflow)?
            .ok_or(crate::aerodynamics::AerodynamicsError::ArithmeticOverflow)?;
        let new_center = new_body
            .world_center_of_mass()
            .map_err(|_| crate::aerodynamics::AerodynamicsError::ArithmeticOverflow)?
            .ok_or(crate::aerodynamics::AerodynamicsError::ArithmeticOverflow)?;
        let world_point = old_center
            .checked_add(
                old_body
                    .state()
                    .transform()
                    .transform_vector(surface.properties().local_application_offset()),
            )
            .ok_or(crate::aerodynamics::AerodynamicsError::ArithmeticOverflow)?;
        let inverse_rotation = new_body
            .state()
            .transform()
            .rotation()
            .conjugate()
            .rotate_vector(
                world_point
                    .checked_sub(new_center)
                    .ok_or(crate::aerodynamics::AerodynamicsError::ArithmeticOverflow)?,
            )
            .ok_or(crate::aerodynamics::AerodynamicsError::ArithmeticOverflow)?;
        let mut properties = surface
            .properties()
            .with_local_application_offset(inverse_rotation)?;
        if let Some(axis) = surface.properties().local_lift_axis() {
            let world_axis = old_body
                .state()
                .transform()
                .rotation()
                .rotate_vector(axis)
                .ok_or(crate::aerodynamics::AerodynamicsError::ArithmeticOverflow)?;
            let local_axis = new_body
                .state()
                .transform()
                .rotation()
                .conjugate()
                .rotate_vector(world_axis)
                .ok_or(crate::aerodynamics::AerodynamicsError::ArithmeticOverflow)?;
            properties =
                properties.with_lift(surface.properties().lift_coefficient(), local_axis)?;
        }
        Ok(surface.with_properties(properties))
    }

    /// Adds an independently managed gravity field.
    pub fn add_gravity(&mut self, gravity: impl Into<Gravity>) -> Result<GravityId, WorldError> {
        if self.world_id == 0 {
            self.world_id = allocate_world_id(&NEXT_WORLD_ID)?;
        }
        let dense = self.gravity_fields.insert(gravity.into())?;
        Ok(GravityId {
            world: self.world_id,
            dense,
        })
    }

    pub fn gravity(&self, id: GravityId) -> Option<Gravity> {
        (id.world == self.world_id)
            .then(|| self.gravity_fields.get(id.dense).copied())
            .flatten()
    }

    pub fn gravity_fields(&self) -> impl Iterator<Item = (GravityId, Gravity)> + '_ {
        self.gravity_fields.iter().map(|(dense, field)| {
            (
                GravityId {
                    world: self.world_id,
                    dense,
                },
                *field,
            )
        })
    }

    pub fn update_gravity(
        &mut self,
        id: GravityId,
        gravity: impl Into<Gravity>,
    ) -> Result<(), WorldError> {
        if id.world != self.world_id {
            return Err(WorldError::InvalidGravityId);
        }
        *self
            .gravity_fields
            .get_mut(id.dense)
            .ok_or(WorldError::InvalidGravityId)? = gravity.into();
        Ok(())
    }

    pub fn remove_gravity(&mut self, id: GravityId) -> Option<Gravity> {
        if id.world != self.world_id {
            return None;
        }
        self.gravity_fields.remove(id.dense)
    }

    pub fn set_collision_config(&mut self, config: CollisionConfig) -> Result<(), CollisionError> {
        self.collision_system.set_config(config)
    }

    pub fn collision_events(&self) -> &[CollisionEvent] {
        self.collision_system.events()
    }

    /// Returns joint-break events committed by the latest successful step.
    pub fn joint_events(&self) -> &[JointBreakEvent] {
        &self.joint_events
    }

    pub fn add_material(&mut self, material: PhysicsMaterial) -> Result<MaterialId, WorldError> {
        self.materials.insert(material).map_err(WorldError::from)
    }

    pub fn material(&self, id: MaterialId) -> Option<&PhysicsMaterial> {
        self.materials.get(id)
    }

    pub fn material_mut(&mut self, id: MaterialId) -> Option<&mut PhysicsMaterial> {
        self.materials.get_mut(id)
    }

    /// Replaces a material and invalidates cached contacts that used it.
    pub fn update_material(
        &mut self,
        id: MaterialId,
        material: PhysicsMaterial,
    ) -> Result<(), WorldError> {
        let current = self
            .materials
            .get_mut(id)
            .ok_or(WorldError::InvalidMaterialId)?;
        if *current == material {
            return Ok(());
        }
        *current = material;
        for (dense, entry) in self.colliders.iter() {
            if entry.collider.material() == Some(id) {
                self.collision_system
                    .invalidate_collider_contacts(ColliderId {
                        world: self.world_id,
                        dense,
                    });
            }
        }
        Ok(())
    }

    pub fn remove_material(&mut self, id: MaterialId) -> Result<PhysicsMaterial, WorldError> {
        if !self.materials.contains(id) {
            return Err(WorldError::InvalidMaterialId);
        }
        if self
            .colliders
            .iter()
            .any(|(_, entry)| entry.collider.material() == Some(id))
        {
            return Err(WorldError::MaterialInUse);
        }
        self.materials
            .remove(id)
            .ok_or(WorldError::InvalidMaterialId)
    }

    pub fn set_collider_material(
        &mut self,
        id: ColliderId,
        material: Option<MaterialId>,
    ) -> Result<(), WorldError> {
        if id.world != self.world_id || self.colliders.get(id.dense).is_none() {
            return Err(WorldError::InvalidColliderId);
        }
        if material.is_some_and(|material| !self.materials.contains(material)) {
            return Err(WorldError::InvalidMaterialId);
        }
        let entry = self
            .colliders
            .get_mut(id.dense)
            .ok_or(WorldError::InvalidColliderId)?;
        entry.collider.set_material(material);
        self.collision_system.mark_spatial_dirty();
        self.collision_system.invalidate_collider_contacts(id);
        Ok(())
    }

    /// Sets whether an attached collider responds physically or only reports intersections.
    pub fn set_collider_mode(
        &mut self,
        id: ColliderId,
        mode: crate::collider::CollisionMode,
    ) -> Result<(), WorldError> {
        if id.world != self.world_id {
            return Err(WorldError::InvalidColliderId);
        }
        let entry = self
            .colliders
            .get_mut(id.dense)
            .ok_or(WorldError::InvalidColliderId)?;
        if entry.collider.mode() != mode {
            entry.collider.set_mode(mode);
            self.collision_system.invalidate_collider_contacts(id);
        }
        Ok(())
    }

    /// Changes collider aerodynamic resolution, invalidating local preparation.
    pub fn set_collider_tessellation(
        &mut self,
        body: BodyId,
        config: crate::aerodynamics::ColliderTessellation,
    ) -> Result<(), WorldError> {
        if body.world != self.world_id {
            return Err(WorldError::InvalidBodyId);
        }
        let entry = self
            .bodies
            .get_mut(body.dense)
            .ok_or(WorldError::InvalidBodyId)?;
        if entry.collider_tessellation != config {
            entry.collider_tessellation = config;
            entry.collider_aerodynamic_mesh = None;
            entry.mesh_aerodynamics = None;
            entry.body.invalidate_collider_base_patches();
            if let Some(caches) = &mut self.assembly_aerodynamics {
                caches.remove(&body);
            }
            self.body_revision = self.body_revision.wrapping_add(1);
        }
        Ok(())
    }

    pub fn add_body(&mut self, body: RigidBody) -> Result<BodyId, WorldError> {
        if self.world_id == 0 {
            self.world_id = allocate_world_id(&NEXT_WORLD_ID)?;
        }
        let dense = self
            .bodies
            .insert(BodyEntry {
                body,
                first_collider: None,
                cluster: None,
                mesh_aerodynamics: None,
                collider_aerodynamic_mesh: None,
                collider_tessellation: crate::aerodynamics::ColliderTessellation::default(),
            })
            .map_err(WorldError::from)?;
        Ok(BodyId {
            world: self.world_id,
            dense,
        })
    }

    pub fn body(&self, id: BodyId) -> Option<&RigidBody> {
        if id.world != self.world_id {
            return None;
        }
        self.bodies.get(id.dense).map(|entry| &entry.body)
    }

    /// Returns mutable access only for bodies whose mass is not cluster-managed.
    pub fn body_mut(&mut self, id: BodyId) -> Option<&mut RigidBody> {
        if id.world != self.world_id || self.bodies.get(id.dense)?.cluster.is_some() {
            return None;
        }
        self.invalidate_body_edit(id);
        self.bodies.get_mut(id.dense).map(|entry| &mut entry.body)
    }

    /// Applies a validated edit to a copy before updating a body in this world.
    /// Cluster-owned mass and motion type must remain unchanged.
    pub fn edit_body(
        &mut self,
        id: BodyId,
        edit: impl FnOnce(&mut BodyMotion) -> Result<(), crate::rigid_body::BodyError>,
    ) -> Result<(), ClusterError> {
        let entry = self
            .bodies
            .get(id.dense)
            .filter(|_| id.world == self.world_id)
            .ok_or(ClusterError::InvalidBodyId)?;
        let mut motion = entry.body.motion();
        edit(&mut motion).map_err(ClusterError::Body)?;
        if entry.cluster.is_some() && entry.body.physics() != motion.physics() {
            return Err(ClusterError::ManagedMass);
        }
        self.invalidate_body_edit(id);
        self.bodies
            .get_mut(id.dense)
            .ok_or(ClusterError::InvalidBodyId)?
            .body
            .set_motion(motion);
        Ok(())
    }

    /// Replaces body state while protecting cluster-managed physical properties.
    pub fn replace_body_state(&mut self, id: BodyId, body: RigidBody) -> Result<(), ClusterError> {
        if id.world != self.world_id {
            return Err(ClusterError::InvalidBodyId);
        }
        let entry = self
            .bodies
            .get(id.dense)
            .ok_or(ClusterError::InvalidBodyId)?;
        if entry.cluster.is_some() && entry.body.physics() != body.physics() {
            return Err(ClusterError::ManagedMass);
        }
        if entry.cluster.is_some()
            && matches!(
                body.aerodynamics(),
                crate::aerodynamics::Aerodynamics::Mesh(_)
                    | crate::aerodynamics::Aerodynamics::VoxelWave(_)
                    | crate::aerodynamics::Aerodynamics::Colliders
            )
        {
            return Err(ClusterError::UnsupportedGeometry);
        }
        self.invalidate_body_edit(id);
        let entry = self
            .bodies
            .get_mut(id.dense)
            .ok_or(ClusterError::InvalidBodyId)?;
        entry.body = body;
        Ok(())
    }

    fn invalidate_body_edit(&mut self, id: BodyId) {
        self.body_revision = self.body_revision.wrapping_add(1);
        self.synchronize_cluster_physics_state(&[id]);
    }

    pub(crate) fn synchronize_cluster_physics_state(&mut self, bodies: &[BodyId]) {
        let mut colliders = Vec::new();
        for &body in bodies {
            let mut next = self
                .bodies
                .get(body.dense)
                .and_then(|entry| entry.first_collider);
            while let Some(id) = next {
                colliders.push(id);
                next = self.colliders.get(id.dense).and_then(|entry| entry.next);
            }
        }
        for collider in colliders {
            self.collision_system.invalidate_collider_contacts(collider);
        }
        if self.collision_system.joint_warm_start_enabled() {
            for (dense, entry) in self.joints.iter() {
                if bodies.contains(&entry.body_a) || bodies.contains(&entry.body_b) {
                    self.collision_system.invalidate_joint_warm_start(JointId {
                        world: self.world_id,
                        dense,
                    });
                }
            }
        }
        self.mark_query_index_dirty();
        self.collision_system.mark_spatial_dirty();
    }

    pub fn body_count(&self) -> usize {
        self.bodies.len()
    }

    /// Adds a constraint between two existing, distinct bodies.
    pub fn add_joint(
        &mut self,
        body_a: BodyId,
        body_b: BodyId,
        joint: Joint,
    ) -> Result<JointId, WorldError> {
        if body_a == body_b || self.body(body_a).is_none() || self.body(body_b).is_none() {
            return Err(WorldError::InvalidJointBodies);
        }
        joint.validate()?;
        let dense = self
            .joints
            .insert(JointEntry {
                target_a: JointTarget::Body(body_a),
                target_b: JointTarget::Body(body_b),
                body_a,
                body_b,
                joint,
            })
            .map_err(WorldError::from)?;
        Ok(JointId {
            world: self.world_id,
            dense,
        })
    }

    /// Adds a constraint between two body or cluster-part targets.
    pub fn add_joint_targets(
        &mut self,
        target_a: JointTarget,
        target_b: JointTarget,
        joint: Joint,
    ) -> Result<JointId, WorldError> {
        let (body_a, frame_a) = self.resolve_joint_target(target_a)?;
        let (body_b, frame_b) = self.resolve_joint_target(target_b)?;
        if target_a == target_b {
            return Err(WorldError::InvalidJointBodies);
        }
        joint.validate()?;
        let frame_a = transform_joint_frame(frame_a, joint.common().frame_a)
            .map_err(|_| WorldError::InvalidJointTarget)?;
        let frame_b = transform_joint_frame(frame_b, joint.common().frame_b)
            .map_err(|_| WorldError::InvalidJointTarget)?;
        let dense = self
            .joints
            .insert(JointEntry {
                target_a,
                target_b,
                body_a,
                body_b,
                joint: joint.with_frames(frame_a, frame_b),
            })
            .map_err(WorldError::from)?;
        Ok(JointId {
            world: self.world_id,
            dense,
        })
    }

    fn resolve_joint_target(
        &self,
        target: JointTarget,
    ) -> Result<(BodyId, crate::transform::Transform), WorldError> {
        match target {
            JointTarget::Body(body) => self
                .body(body)
                .map(|_| (body, crate::transform::Transform::identity()))
                .ok_or(WorldError::InvalidJointTarget),
            JointTarget::Part(part) => self
                .cluster_part(part)
                .and_then(|entry| {
                    self.cluster(entry.body())
                        .map(|_| (entry.body(), entry.local_transform()))
                })
                .ok_or(WorldError::InvalidJointTarget),
        }
    }

    /// Returns the configuration of a live joint in this world.
    pub fn joint(&self, id: JointId) -> Option<&Joint> {
        if id.world != self.world_id {
            return None;
        }
        self.joints.get(id.dense).map(|entry| &entry.joint)
    }

    /// Returns the two bodies connected by a live joint.
    pub fn joint_bodies(&self, id: JointId) -> Option<(BodyId, BodyId)> {
        if id.world != self.world_id {
            return None;
        }
        self.joints
            .get(id.dense)
            .map(|entry| (entry.body_a, entry.body_b))
    }

    pub fn joint_targets(&self, id: JointId) -> Option<(JointTarget, JointTarget)> {
        if id.world != self.world_id {
            return None;
        }
        self.joints
            .get(id.dense)
            .map(|entry| (entry.target_a, entry.target_b))
    }

    /// Replaces a joint's settings while retaining its ID and attached bodies.
    pub fn update_joint(&mut self, id: JointId, joint: Joint) -> Result<(), WorldError> {
        joint.validate()?;
        if id.world != self.world_id {
            return Err(WorldError::InvalidJointId);
        }
        let entry = self
            .joints
            .get_mut(id.dense)
            .ok_or(WorldError::InvalidJointId)?;
        entry.joint = joint;
        self.collision_system.invalidate_joint_warm_start(id);
        Ok(())
    }

    /// Replaces a joint using frames expressed in the current target spaces.
    pub fn update_joint_targets(&mut self, id: JointId, joint: Joint) -> Result<(), WorldError> {
        joint.validate()?;
        if id.world != self.world_id {
            return Err(WorldError::InvalidJointId);
        }
        let (target_a, target_b) = self
            .joints
            .get(id.dense)
            .map(|entry| (entry.target_a, entry.target_b))
            .ok_or(WorldError::InvalidJointId)?;
        let (_, frame_a_target) = self.resolve_joint_target(target_a)?;
        let (_, frame_b_target) = self.resolve_joint_target(target_b)?;
        let frame_a = transform_joint_frame(frame_a_target, joint.common().frame_a)
            .map_err(|_| WorldError::InvalidJointTarget)?;
        let frame_b = transform_joint_frame(frame_b_target, joint.common().frame_b)
            .map_err(|_| WorldError::InvalidJointTarget)?;
        let entry = self
            .joints
            .get_mut(id.dense)
            .ok_or(WorldError::InvalidJointId)?;
        entry.joint = joint.with_frames(frame_a, frame_b);
        self.collision_system.invalidate_joint_warm_start(id);
        Ok(())
    }

    /// Enables or disables a live joint without changing its settings.
    pub fn set_joint_enabled(&mut self, id: JointId, enabled: bool) -> Result<(), WorldError> {
        if id.world != self.world_id {
            return Err(WorldError::InvalidJointId);
        }
        self.joints
            .get_mut(id.dense)
            .ok_or(WorldError::InvalidJointId)?
            .joint
            .set_enabled(enabled);
        if !enabled {
            self.collision_system.invalidate_joint_warm_start(id);
        }
        Ok(())
    }

    /// Removes a live joint and returns its final configuration.
    pub fn remove_joint(&mut self, id: JointId) -> Option<Joint> {
        if id.world != self.world_id {
            return None;
        }
        let removed = self.joints.remove(id.dense).map(|entry| entry.joint);
        if removed.is_some() {
            self.collision_system.invalidate_joint_warm_start(id);
        }
        removed
    }

    pub fn joint_count(&self) -> usize {
        self.joints.len()
    }

    /// Iterates over joints in dense storage order, which may change after removal.
    pub fn joints(&self) -> impl Iterator<Item = (JointId, BodyId, BodyId, &Joint)> {
        self.joints.iter().map(|(dense, entry)| {
            (
                JointId {
                    world: self.world_id,
                    dense,
                },
                entry.body_a,
                entry.body_b,
                &entry.joint,
            )
        })
    }

    /// Adds a persistent force source to an existing dynamic body.
    pub fn add_force_source(
        &mut self,
        body: BodyId,
        local_point: Vec3,
        local_force: Vec3,
    ) -> Result<ForceSourceId, ForceSourceError> {
        let Some(body_state) = self.body(body) else {
            return Err(ForceSourceError::InvalidBodyId);
        };
        if body_state.physics().body_type() != BodyType::Dynamic {
            return Err(ForceSourceError::BodyMustBeDynamic);
        }
        validate_force_source_values(local_point, local_force)?;
        let dense = self
            .force_sources
            .insert(ForceSource {
                body,
                target: ForceTarget::Body(body),
                local_point,
                local_force,
                enabled: true,
            })
            .map_err(|_| ForceSourceError::StorageCapacityExceeded)?;
        Ok(ForceSourceId {
            world: self.world_id,
            dense,
        })
    }

    /// Adds a persistent force source attached to a part of a rigid cluster.
    pub fn add_force_source_for_part(
        &mut self,
        part: PartId,
        local_point: Vec3,
        local_force: Vec3,
    ) -> Result<ForceSourceId, ForceSourceError> {
        let part_entry = self
            .cluster_part(part)
            .ok_or(ForceSourceError::InvalidPartId)?;
        let body = part_entry.body();
        let Some(cluster) = self.cluster(body) else {
            return Err(ForceSourceError::InvalidPartId);
        };
        if !cluster.parts().contains(&part) {
            return Err(ForceSourceError::InvalidPartId);
        }
        let Some(body_state) = self.body(body) else {
            return Err(ForceSourceError::InvalidBodyId);
        };
        if body_state.physics().body_type() != BodyType::Dynamic {
            return Err(ForceSourceError::BodyMustBeDynamic);
        }
        validate_force_source_values(local_point, local_force)?;
        let dense = self
            .force_sources
            .insert(ForceSource {
                body,
                target: ForceTarget::Part(part),
                local_point,
                local_force,
                enabled: true,
            })
            .map_err(|_| ForceSourceError::StorageCapacityExceeded)?;
        Ok(ForceSourceId {
            world: self.world_id,
            dense,
        })
    }

    /// Looks up a force source by its world-scoped ID.
    pub fn force_source(&self, id: ForceSourceId) -> Option<&ForceSource> {
        (id.world == self.world_id)
            .then(|| self.force_sources.get(id.dense))
            .flatten()
    }

    /// Iterates over persistent force sources in dense storage order.
    pub fn force_sources(&self) -> impl Iterator<Item = (ForceSourceId, &ForceSource)> {
        self.force_sources.iter().map(|(dense, source)| {
            (
                ForceSourceId {
                    world: self.world_id,
                    dense,
                },
                source,
            )
        })
    }

    /// Replaces the local application point and force vector of a source.
    pub fn set_force_source(
        &mut self,
        id: ForceSourceId,
        local_point: Vec3,
        local_force: Vec3,
    ) -> Result<(), ForceSourceError> {
        validate_force_source_values(local_point, local_force)?;
        let source = self
            .force_source_mut(id)
            .ok_or(ForceSourceError::InvalidForceSourceId)?;
        source.local_point = local_point;
        source.local_force = local_force;
        Ok(())
    }

    /// Enables or disables a source without removing it.
    pub fn set_force_source_enabled(
        &mut self,
        id: ForceSourceId,
        enabled: bool,
    ) -> Result<(), ForceSourceError> {
        self.force_source_mut(id)
            .ok_or(ForceSourceError::InvalidForceSourceId)?
            .enabled = enabled;
        Ok(())
    }

    /// Removes a source and returns its last configuration.
    pub fn remove_force_source(
        &mut self,
        id: ForceSourceId,
    ) -> Result<ForceSource, ForceSourceError> {
        if id.world != self.world_id {
            return Err(ForceSourceError::InvalidForceSourceId);
        }
        self.force_sources
            .remove(id.dense)
            .ok_or(ForceSourceError::InvalidForceSourceId)
    }

    fn force_source_mut(&mut self, id: ForceSourceId) -> Option<&mut ForceSource> {
        if id.world != self.world_id {
            return None;
        }
        self.force_sources.get_mut(id.dense)
    }

    /// Advances every body by one fixed timestep, atomically committing the results.
    pub fn step(&mut self, dt: f64) -> Result<(), StepError> {
        self.step_with_executor(dt, &SequentialExecutor, 1)
    }

    /// Advances one step using the supplied statically dispatched task executor.
    pub fn step_with_executor<E: StepExecutor>(
        &mut self,
        dt: f64,
        executor: &E,
        worker_count: usize,
    ) -> Result<(), StepError> {
        if worker_count <= 1 {
            return self.step_with_executor_inner(dt, executor, worker_count);
        }
        match catch_unwind(AssertUnwindSafe(|| {
            self.step_with_executor_inner(dt, executor, worker_count)
        })) {
            Ok(result) => result,
            Err(_) => {
                self.collision_system.abort_step();
                self.rollback_gravity_updates();
                self.rollback_aerodynamics_updates();
                self.rollback_force_source_updates();
                Err(StepError::Collision(CollisionError::WorkerFailed))
            }
        }
    }

    fn step_with_executor_inner<E: StepExecutor>(
        &mut self,
        dt: f64,
        executor: &E,
        worker_count: usize,
    ) -> Result<(), StepError> {
        if !dt.is_finite() || dt <= 0.0 {
            return Err(StepError::InvalidDeltaTime);
        }
        self.with_force_source_updates(|world| {
            world.prepare_aerodynamics_updates(executor, worker_count, dt)?;
            world.prepare_gravity_updates(executor, worker_count)?;
            world.step_prepared(dt, executor, worker_count)
        })
    }

    fn with_force_source_updates<T>(
        &mut self,
        operation: impl FnOnce(&mut Self) -> Result<T, StepError>,
    ) -> Result<T, StepError> {
        self.pending_step_diagnostics = StepDiagnostics::default();
        self.contact_impulses_pending.clear();
        self.contact_impulses_omitted_pending = 0;
        self.contact_ccd_clipped_pending = false;
        self.validate_collision_inputs()
            .map_err(StepError::Collision)?;
        self.cluster_load_states_pending.clear();
        let cluster_count = self
            .bodies
            .iter()
            .filter(|(_, entry)| entry.cluster.is_some())
            .count();
        self.cluster_load_states_pending
            .try_reserve(cluster_count)
            .map_err(|_| StepError::Collision(CollisionError::SpatialIndexCapacityExceeded))?;
        self.cluster_load_parts_pending.clear();
        self.cluster_load_parts_pending
            .try_reserve(self.parts.iter().count())
            .map_err(|_| StepError::Collision(CollisionError::SpatialIndexCapacityExceeded))?;
        self.cluster_load_welds_pending.clear();
        self.cluster_load_welds_pending
            .try_reserve(self.welds.iter().count())
            .map_err(|_| StepError::Collision(CollisionError::SpatialIndexCapacityExceeded))?;
        let no_world_load_paths =
            self.joints.iter().next().is_none() && self.chunks.iter().next().is_none();
        for (dense, entry) in self.bodies.iter() {
            if let Some(cluster) = &entry.cluster {
                let body = &entry.body;
                let sources_certified = self.force_sources.iter().all(|(_, source)| {
                    !source.enabled
                        || source.body
                            != BodyId {
                                world: self.world_id,
                                dense,
                            }
                        || matches!(source.target, ForceTarget::Part(_))
                });
                let part_surfaces_certified = no_world_load_paths
                    && sources_certified
                    && body.physics().body_type() == BodyType::Dynamic
                    && body.forces().force() == Vec3::ZERO
                    && body.forces().torque() == Vec3::ZERO
                    && matches!(body.aerodynamics(), crate::aerodynamics::Aerodynamics::None)
                    && body
                        .aerodynamic_surfaces
                        .iter()
                        .all(|surface| matches!(surface.target(), AerodynamicTarget::Part(_)))
                    && body.base_pressure_patches.is_empty()
                    && body.potential_sphere.is_none()
                    && cluster.parts.iter().all(|part_id| {
                        self.parts.get(part_id.dense).is_some()
                            && (matches!(self.atmosphere, Atmosphere::Vacuum)
                                || !self
                                    .part_aerodynamic_geometry
                                    .as_ref()
                                    .and_then(|geometry| geometry.get(part_id))
                                    .is_some_and(|geometry| {
                                        !matches!(
                                            geometry,
                                            assembly_aerodynamics::PartGeometry::Mesh(_)
                                                | assembly_aerodynamics::PartGeometry::Wave(_)
                                                | assembly_aerodynamics::PartGeometry::WaveSourceMesh(_)
                                        )
                                    }))
                    });
                let has_geometry = cluster.parts.iter().any(|part| {
                    self.part_aerodynamic_geometry
                        .as_ref()
                        .is_some_and(|geometry| geometry.contains_key(part))
                });
                let has_pressure_geometry = cluster.parts.iter().any(|part| {
                    self.part_aerodynamic_geometry
                        .as_ref()
                        .and_then(|geometry| geometry.get(part))
                        .is_some_and(|geometry| {
                            matches!(geometry, assembly_aerodynamics::PartGeometry::Mesh(_))
                        })
                });
                let part_wings_certified = part_surfaces_certified
                    && !has_geometry
                    && body.aerodynamic_surfaces.is_empty();
                let part_hull_certified = part_wings_certified
                    && cluster.parts.iter().all(|id| {
                        self.parts
                            .get(id.dense)
                            .is_some_and(|part| part.aerodynamic_wings.is_empty())
                    });
                let part_sources_certified = part_hull_certified
                    && cluster.parts.iter().all(|id| {
                        self.parts
                            .get(id.dense)
                            .is_some_and(|part| part.aerodynamic_hull.is_empty())
                    });
                let force_free_certified = part_sources_certified
                    && self.central_gravity.is_none()
                    && self.gravity_fields.is_empty()
                    && self.force_sources.iter().all(|(_, source)| {
                        !source.enabled
                            || source.body
                                != BodyId {
                                    world: self.world_id,
                                    dense,
                                }
                    });
                self.cluster_load_states_pending.push(ClusterLoadState {
                    body: BodyId {
                        world: self.world_id,
                        dense,
                    },
                    root: cluster.root,
                    state: *entry.body.state(),
                    mass: body.physics().mass(),
                    force_free_certified,
                    part_sources_certified,
                    part_hull_certified,
                    part_wings_certified,
                    part_surfaces_certified,
                    collision_loads_uncertified: false,
                    part_surface_count: body.aerodynamic_surfaces.len(),
                    vacuum: matches!(self.atmosphere, Atmosphere::Vacuum),
                    part_geometry: has_pressure_geometry,
                });
            }
        }
        for (dense, part) in self.parts.iter() {
            self.cluster_load_parts_pending.push(ClusterLoadPart {
                part: PartId {
                    world: self.world_id,
                    dense,
                },
                body: part.body,
                local_transform: part.local_transform,
                mass: part.mass,
            });
        }
        for (dense, weld) in self.welds.iter() {
            self.cluster_load_welds_pending.push(ClusterLoadWeld {
                weld: WeldId {
                    world: self.world_id,
                    dense,
                },
                value: *weld,
            });
        }
        if let Err((source_id, cause)) = self.prepare_force_source_updates() {
            return Err(StepError::ForceSource { source_id, cause });
        }

        let result = operation(self);
        if result.is_err() {
            self.rollback_gravity_updates();
            self.rollback_aerodynamics_updates();
            self.rollback_force_source_updates();
        } else {
            self.step_diagnostics = self.pending_step_diagnostics;
            if let Some(workspaces) = &mut self.owner_pressure {
                workspaces.retain(|_, workspace| workspace.commit());
            }
            std::mem::swap(
                &mut self.part_force_source_published,
                &mut self.part_force_source_pending,
            );
            std::mem::swap(
                &mut self.part_gravity_published,
                &mut self.part_gravity_pending,
            );
            std::mem::swap(&mut self.part_hull_published, &mut self.part_hull_pending);
            std::mem::swap(&mut self.part_wing_published, &mut self.part_wing_pending);
            std::mem::swap(
                &mut self.simple_surface_published,
                &mut self.simple_surface_pending,
            );
            std::mem::swap(&mut self.raw_simple_published, &mut self.raw_simple_pending);
            std::mem::swap(
                &mut self.contact_impulses_published,
                &mut self.contact_impulses_pending,
            );
            std::mem::swap(
                &mut self.contact_impulses_omitted_published,
                &mut self.contact_impulses_omitted_pending,
            );
            std::mem::swap(
                &mut self.contact_ccd_clipped_published,
                &mut self.contact_ccd_clipped_pending,
            );
            std::mem::swap(
                &mut self.cluster_load_states_published,
                &mut self.cluster_load_states_pending,
            );
            std::mem::swap(
                &mut self.cluster_load_parts_published,
                &mut self.cluster_load_parts_pending,
            );
            std::mem::swap(
                &mut self.cluster_load_welds_published,
                &mut self.cluster_load_welds_pending,
            );
            self.recycle_aerodynamics_updates();
            self.clear_gravity_updates();
            self.clear_force_source_updates();
        }
        result
    }

    /// Advances one full step through reusable prediction shards and the supplied executor.
    ///
    /// Unlike the low-level `fill_step_shards` and `commit_step_shards` pair, this method
    /// prepares persistent force sources and rolls prepared force and gravity back on failure.
    #[doc(hidden)]
    pub fn step_with_shards<E: StepExecutor>(
        &mut self,
        dt: f64,
        shards: &mut [Vec<BodyStep>],
        executor: &E,
        worker_count: usize,
    ) -> Result<(), StepError> {
        if self.body_count() == 0 {
            return self.step_with_executor_inner(dt, executor, worker_count);
        }
        let result = catch_unwind(AssertUnwindSafe(|| {
            self.step_with_shards_inner(dt, shards, executor, worker_count)
        }));
        match result {
            Ok(result) => result,
            Err(_) => {
                self.collision_system.abort_step();
                self.rollback_gravity_updates();
                self.rollback_aerodynamics_updates();
                self.rollback_force_source_updates();
                Err(StepError::Collision(CollisionError::WorkerFailed))
            }
        }
    }

    fn step_with_shards_inner<E: StepExecutor>(
        &mut self,
        dt: f64,
        shards: &mut [Vec<BodyStep>],
        executor: &E,
        worker_count: usize,
    ) -> Result<(), StepError> {
        if !dt.is_finite() || dt <= 0.0 {
            return Err(StepError::InvalidDeltaTime);
        }

        self.with_force_source_updates(|world| {
            if shards.is_empty() {
                return Err(StepError::InvalidStepResults);
            }
            // Prepare persistent aerodynamic forces through the same path as the
            // ordinary step.  In particular, Mesh requires prepared surface
            // patches and must not be evaluated with a missing patch set.
            world.prepare_aerodynamics_updates(executor, worker_count, dt)?;
            world.fill_step_shards_with_executor_at_dt(shards, executor, Some(dt), false)?;
            executor
                .for_each_chunk_mut(shards, 1, |_, chunk| {
                    for shard in chunk {
                        for step in shard {
                            if step.integrate(dt).is_err() {
                                break;
                            }
                        }
                    }
                })
                .map_err(|_| StepError::Collision(CollisionError::WorkerFailed))?;
            world.commit_step_shards_inner(shards, executor, worker_count, true)
        })
    }

    fn step_prepared<E: StepExecutor>(
        &mut self,
        dt: f64,
        executor: &E,
        worker_count: usize,
    ) -> Result<(), StepError> {
        self.collision_system.begin_step();
        #[cfg(feature = "bench-internals")]
        self.collision_system
            .record_energy_before_integration(self.energy_components());
        if let Err(error) = self.prepare_joint_collision_pairs() {
            self.collision_system.abort_step();
            return Err(StepError::Collision(error));
        }

        #[cfg(feature = "bench-internals")]
        let mut integrated_energy = crate::joint::diagnostics::EnergyComponents::default();
        for (dense_id, entry) in self.bodies.iter() {
            let body_id = BodyId {
                world: self.world_id,
                dense: dense_id,
            };
            match predict(&entry.body, dt) {
                Ok(state) => {
                    #[cfg(feature = "bench-internals")]
                    integrated_energy.add_assign(crate::collision::solver::body_energy_components(
                        &entry.body,
                        state,
                        self.central_gravity,
                    ));
                    self.collision_system.stage_prediction(body_id, state)
                }
                Err(cause) => {
                    self.collision_system.abort_step();
                    return Err(StepError::BodyIntegration { body_id, cause });
                }
            }
        }
        #[cfg(feature = "bench-internals")]
        self.collision_system
            .record_energy_after_integration(integrated_energy);

        let worker_count = worker_count.max(1);
        if let Err(error) = self.prepare_collision_candidates(dt, worker_count, executor) {
            self.collision_system.abort_step();
            return Err(StepError::Collision(error));
        }
        if let Err(error) = self.process_collision_candidates(worker_count, executor, dt) {
            self.collision_system.abort_step();
            return Err(StepError::Collision(error));
        }
        if let Err(error) = self.prepare_joint_break_commit() {
            self.collision_system.abort_step();
            return Err(StepError::Collision(error));
        }
        if let Err(error) = self.stage_contact_impulses() {
            self.collision_system.abort_step();
            return Err(StepError::Collision(error));
        }
        let splits = match self.stage_automatic_weld_splits() {
            Ok(splits) => splits,
            Err(error) => {
                self.collision_system.abort_step();
                return Err(error);
            }
        };

        #[cfg(feature = "bench-internals")]
        let commit_started = Instant::now();
        self.commit_predicted_states();
        if let Some(splits) = splits {
            self.finish_weld_splits(splits);
        }
        self.commit_joint_breaks();
        self.collision_system.commit_step();
        #[cfg(feature = "bench-internals")]
        self.collision_system
            .record_commit(commit_started.elapsed());
        Ok(())
    }

    fn stage_contact_impulses(&mut self) -> Result<(), CollisionError> {
        self.contact_impulses_pending.clear();
        self.contact_impulses_omitted_pending = 0;
        let context = &self.collision_system.context;
        self.pending_step_diagnostics
            .collision_ccd_limit_clipped_bodies = context
            .ccd_clipped_bodies
            .iter()
            .filter(|&&clipped| clipped)
            .count();
        self.contact_ccd_clipped_pending = self
            .pending_step_diagnostics
            .collision_ccd_limit_clipped_bodies
            != 0;
        // A manifold can change motion without a recorded velocity impulse.
        // Mark its owners and every clipped CCD body, independently of the
        // impulse snapshot. Unknown ownership conservatively invalidates all.
        let indices = self.cluster_load_indices.get_or_insert_with(HashMap::new);
        indices.clear();
        indices
            .try_reserve(self.cluster_load_states_pending.len())
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        for (index, state) in self.cluster_load_states_pending.iter().enumerate() {
            indices.insert(state.body, index);
        }
        let mut unknown = context.ccd_clipped_bodies.len() != context.predicted_states.len()
            && !context.ccd_clipped_bodies.is_empty();
        for (clipped, (body, _)) in context
            .ccd_clipped_bodies
            .iter()
            .zip(&context.predicted_states)
        {
            if *clipped {
                if let Some(&index) = indices.get(body) {
                    self.cluster_load_states_pending[index].invalidate_collision_loads();
                }
            }
        }
        for manifold in context
            .ccd_solved_manifolds
            .iter()
            .chain(&context.manifolds)
        {
            for collider in [manifold.pair.a, manifold.pair.b] {
                if let Some(body) = self
                    .colliders
                    .get(collider.dense)
                    .filter(|_| collider.world == self.world_id)
                    .map(|entry| entry.body)
                {
                    if let Some(&index) = indices.get(&body) {
                        self.cluster_load_states_pending[index].invalidate_collision_loads();
                    }
                } else {
                    unknown = true;
                }
            }
        }
        if unknown {
            for state in &mut self.cluster_load_states_pending {
                state.invalidate_collision_loads();
            }
        }
        let count = context
            .ccd_solved_manifolds
            .iter()
            .chain(&context.manifolds)
            .try_fold(0usize, |count, manifold| count.checked_add(manifold.len()))
            .ok_or(CollisionError::SpatialIndexCapacityExceeded)?;
        self.contact_impulses_pending
            .try_reserve(count)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        for (ccd, manifolds) in [
            (true, &context.ccd_solved_manifolds),
            (false, &context.manifolds),
        ] {
            for manifold in manifolds {
                let parts = [
                    self.collider_part(manifold.pair.a),
                    self.collider_part(manifold.pair.b),
                ];
                for contact in manifold.points() {
                    if let (Some(points), Some(impulse_on_b)) =
                        (contact.velocity_points, contact.velocity_impulse)
                    {
                        self.contact_impulses_pending.push(ContactImpulse {
                            pair: manifold.pair,
                            parts,
                            points,
                            velocity_transforms: contact.velocity_transforms,
                            isolated_velocity_states: contact.isolated_velocity_states,
                            impulse_on_b,
                            ccd,
                        });
                    } else {
                        self.contact_impulses_omitted_pending += 1;
                    }
                }
            }
        }
        Ok(())
    }

    fn prepare_joint_break_commit(&mut self) -> Result<(), CollisionError> {
        self.joint_events
            .try_reserve(self.collision_system.joint_break_events().len())
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)
    }

    fn commit_joint_breaks(&mut self) {
        let world_id = self.world_id;
        let break_events = self.collision_system.joint_break_events();
        let joints = &mut self.joints;
        let committed_events = &mut self.joint_events;
        committed_events.clear();
        for event in break_events.iter().copied() {
            let id = event.joint();
            if id.world == world_id && joints.remove(id.dense).is_some() {
                committed_events.push(event);
            }
        }
    }

    fn prepare_force_source_updates(&mut self) -> Result<(), (ForceSourceId, ForceSourceError)> {
        self.clear_force_source_updates();
        if self.force_sources.is_empty() {
            return Ok(());
        }

        for (dense, source) in self.force_sources.iter() {
            let source_id = ForceSourceId {
                world: self.world_id,
                dense,
            };
            let source = *source;
            if !source.enabled {
                continue;
            }
            let Some(body) = self.body(source.body).map(RigidBody::motion) else {
                self.force_source_body_updates.clear();
                if let Some(indices) = &mut self.force_source_body_indices {
                    indices.clear();
                }
                return Err((source_id, ForceSourceError::InvalidBodyId));
            };
            if body.physics().body_type() != BodyType::Dynamic {
                self.force_source_body_updates.clear();
                if let Some(indices) = &mut self.force_source_body_indices {
                    indices.clear();
                }
                return Err((source_id, ForceSourceError::BodyMustBeDynamic));
            }

            let index_active = self
                .force_source_body_indices
                .as_ref()
                .is_some_and(|indices| !indices.is_empty());
            let update_index = if index_active {
                self.force_source_body_indices
                    .as_ref()
                    .and_then(|indices| indices.get(&source.body).copied())
            } else {
                self.force_source_body_updates
                    .iter()
                    .position(|update| update.id == source.body)
            };
            let update_index = match update_index {
                Some(index) => index,
                None => {
                    if self.force_source_body_updates.try_reserve(1).is_err() {
                        self.force_source_body_updates.clear();
                        if let Some(indices) = &mut self.force_source_body_indices {
                            indices.clear();
                        }
                        return Err((source_id, ForceSourceError::StorageCapacityExceeded));
                    }
                    if index_active {
                        let map_reserve_failed = self
                            .force_source_body_indices
                            .as_mut()
                            .is_none_or(|indices| indices.try_reserve(1).is_err());
                        if map_reserve_failed {
                            self.force_source_body_updates.clear();
                            if let Some(indices) = &mut self.force_source_body_indices {
                                indices.clear();
                            }
                            return Err((source_id, ForceSourceError::StorageCapacityExceeded));
                        }
                    }
                    let index = self.force_source_body_updates.len();
                    self.force_source_body_updates.push(ForceSourceBodyUpdate {
                        id: source.body,
                        original: body,
                        prepared: body,
                    });
                    if index_active {
                        if let Some(indices) = &mut self.force_source_body_indices {
                            indices.insert(source.body, index);
                        }
                    } else if self.force_source_body_updates.len()
                        > FORCE_SOURCE_BODY_LINEAR_SEARCH_LIMIT
                    {
                        let indices = self
                            .force_source_body_indices
                            .get_or_insert_with(HashMap::new);
                        if indices
                            .try_reserve(self.force_source_body_updates.len())
                            .is_err()
                        {
                            self.force_source_body_updates.clear();
                            indices.clear();
                            return Err((source_id, ForceSourceError::StorageCapacityExceeded));
                        }
                        for (index, update) in self.force_source_body_updates.iter().enumerate() {
                            indices.insert(update.id, index);
                        }
                    }
                    index
                }
            };

            let transform = match source.target {
                ForceTarget::Body(target) if target == source.body => self
                    .force_source_body_updates[update_index]
                    .prepared
                    .state()
                    .transform(),
                ForceTarget::Part(part) => {
                    let Some(part_entry) = self.cluster_part(part) else {
                        self.force_source_body_updates.clear();
                        if let Some(indices) = &mut self.force_source_body_indices {
                            indices.clear();
                        }
                        return Err((source_id, ForceSourceError::InvalidPartId));
                    };
                    if part_entry.body() != source.body {
                        self.force_source_body_updates.clear();
                        if let Some(indices) = &mut self.force_source_body_indices {
                            indices.clear();
                        }
                        return Err((source_id, ForceSourceError::InvalidPartId));
                    }
                    self.force_source_body_updates[update_index]
                        .prepared
                        .state()
                        .transform()
                        .compose(&part_entry.local_transform())
                        .map_err(|_| (source_id, ForceSourceError::ArithmeticOverflow))?
                }
                ForceTarget::Body(_) => {
                    self.force_source_body_updates.clear();
                    if let Some(indices) = &mut self.force_source_body_indices {
                        indices.clear();
                    }
                    return Err((source_id, ForceSourceError::InvalidBodyId));
                }
            };
            let body = &mut self.force_source_body_updates[update_index].prepared;
            let force = transform.transform_vector(source.local_force);
            let point = transform.transform_point(source.local_point);
            if let ForceTarget::Part(part) = source.target {
                let center = body
                    .world_center_of_mass()
                    .map_err(|_| (source_id, ForceSourceError::ArithmeticOverflow))?
                    .ok_or((source_id, ForceSourceError::ArithmeticOverflow))?;
                let moment = point
                    .checked_sub(center)
                    .and_then(|arm| arm.checked_cross(force))
                    .ok_or((source_id, ForceSourceError::ArithmeticOverflow))?;
                self.part_force_source_pending
                    .try_reserve(1)
                    .map_err(|_| (source_id, ForceSourceError::StorageCapacityExceeded))?;
                self.part_force_source_pending.push(PartForceSourceWrench {
                    source: source_id,
                    part,
                    body: source.body,
                    force,
                    moment,
                });
            }
            if let Err(error) = body.apply_force_at_point(force, point) {
                self.force_source_body_updates.clear();
                if let Some(indices) = &mut self.force_source_body_indices {
                    indices.clear();
                }
                return Err((source_id, ForceSourceError::from(error)));
            }
        }

        for update in &self.force_source_body_updates {
            if let Some(entry) = self
                .bodies
                .get_mut(update.id.dense)
                .filter(|_| update.id.world == self.world_id)
            {
                entry.body.set_motion(update.prepared);
            }
        }
        Ok(())
    }

    fn clear_force_source_updates(&mut self) {
        self.force_source_body_updates.clear();
        self.part_force_source_pending.clear();
        if let Some(indices) = &mut self.force_source_body_indices {
            indices.clear();
        }
    }

    fn apply_aerodynamics(
        atmosphere: Atmosphere,
        primary: &crate::aerodynamics::Aerodynamics,
        surfaces: &[AerodynamicSurface],
        body_id: BodyId,
        body: &mut BodyMotion,
        parts: &DensePool<ClusterPart>,
        inputs: AerodynamicsInputs<'_>,
    ) -> Result<(), StepError> {
        let AerodynamicsInputs {
            mesh_sound_speed,
            cluster,
            dt,
            mesh_patches,
            sphere_patches,
            voxel_wave,
            assembly_wave,
            wing_visibility,
            base_pressure_patches,
            mut part_hull_wrenches,
            mut part_wing_wrenches,
            simple_surface_correction,
            mut raw_simple_surfaces,
            applied_mesh_pressure,
        } = inputs;
        if matches!(atmosphere, Atmosphere::Vacuum)
            || body.physics().body_type() != BodyType::Dynamic
        {
            return Ok(());
        }
        if !base_pressure_patches.is_empty()
            && (cluster.is_some()
                || !matches!(
                    primary,
                    crate::aerodynamics::Aerodynamics::Mesh(_)
                        | crate::aerodynamics::Aerodynamics::Colliders
                ))
        {
            return Err(StepError::Aerodynamics {
                body_id,
                cause: crate::aerodynamics::AerodynamicsError::UnsupportedModel,
            });
        }
        let error = || StepError::Aerodynamics {
            body_id,
            cause: crate::aerodynamics::AerodynamicsError::ArithmeticOverflow,
        };
        let center = body
            .world_center_of_mass()
            .map_err(|_| error())?
            .ok_or_else(error)?;
        let primary_surface = match primary {
            crate::aerodynamics::Aerodynamics::None
            | crate::aerodynamics::Aerodynamics::Wing(_)
            | crate::aerodynamics::Aerodynamics::Wings(_)
            | crate::aerodynamics::Aerodynamics::Hull(_)
            | crate::aerodynamics::Aerodynamics::VoxelWave(_) => None,
            crate::aerodynamics::Aerodynamics::Simple(properties) => {
                Some(AerodynamicSurface::new(*properties))
            }
            crate::aerodynamics::Aerodynamics::Mesh(_)
            | crate::aerodynamics::Aerodynamics::Colliders => None,
        };
        let primary_atmosphere = if mesh_patches.is_some()
            || matches!(
                primary,
                crate::aerodynamics::Aerodynamics::Mesh(_)
                    | crate::aerodynamics::Aerodynamics::Colliders
            ) {
            Some(
                atmosphere
                    .state_at(center)
                    .map_err(|cause| StepError::Aerodynamics { body_id, cause })?,
            )
            .filter(|state| state.density() != 0.0)
        } else {
            None
        };
        if let Some(primary_atmosphere) = primary_atmosphere {
            let flow = crate::aerodynamics::pressure::PressureFlow {
                atmosphere: primary_atmosphere,
                sound_speed: primary_atmosphere.sound_speed().unwrap_or(mesh_sound_speed),
                transform: body.state().transform(),
                center_of_mass: body
                    .state()
                    .transform()
                    .inverse()
                    .map_err(|_| error())?
                    .transform_point(center),
                linear_velocity: body.state().linear_velocity(),
                angular_velocity: body.state().angular_velocity(),
            };
            let wrench = if let Some(patches) = sphere_patches {
                crate::aerodynamics::pressure::evaluate_potential_sphere_patches(patches, flow)
            } else {
                crate::aerodynamics::pressure::evaluate_prepared_mesh(
                    mesh_patches.ok_or_else(error)?,
                    flow,
                )
            }
            .map_err(|cause| StepError::Aerodynamics {
                body_id,
                cause: crate::aerodynamics::AerodynamicsError::Pressure(cause),
            })?;
            body.apply_force(wrench.force).map_err(|_| error())?;
            body.apply_torque(wrench.moment).map_err(|_| error())?;
            if sphere_patches.is_none()
                && let Some(applied) = applied_mesh_pressure
            {
                *applied = Some(wrench);
            }
            for &(patch, base) in base_pressure_patches {
                let wrench =
                    patch
                        .evaluate_base(flow, base)
                        .map_err(|cause| StepError::Aerodynamics {
                            body_id,
                            cause: crate::aerodynamics::AerodynamicsError::Pressure(cause),
                        })?;
                body.apply_force(wrench.force).map_err(|_| error())?;
                body.apply_torque(wrench.moment).map_err(|_| error())?;
            }
        }
        let primary_hull: &[crate::aerodynamics::hull::HullSection] = match primary {
            crate::aerodynamics::Aerodynamics::Hull(sections) => sections,
            _ => &[],
        };
        if let crate::aerodynamics::Aerodynamics::VoxelWave(model) = primary
            && voxel_wave.is_none()
        {
            let point = body
                .state()
                .transform()
                .transform_point(model.section.local_transform().translation());
            let state = atmosphere
                .state_at(point)
                .map_err(|cause| StepError::Aerodynamics { body_id, cause })?;
            if state.density() != 0.0 {
                return Err(StepError::Aerodynamics {
                    body_id,
                    cause: crate::aerodynamics::AerodynamicsError::UnsupportedModel,
                });
            }
        }
        for section in primary_hull.iter().chain(voxel_wave) {
            let transform = body.state().transform();
            let point = transform.transform_point(section.local_transform().translation());
            if !point.is_finite() {
                return Err(error());
            }
            let state = atmosphere
                .state_at(point)
                .map_err(|cause| StepError::Aerodynamics { body_id, cause })?;
            let wrench = section
                .evaluate(crate::aerodynamics::pressure::PressureFlow {
                    atmosphere: state,
                    sound_speed: state.sound_speed().unwrap_or(mesh_sound_speed),
                    transform,
                    center_of_mass: body.physics().mass().ok_or_else(error)?.center_of_mass(),
                    linear_velocity: body.state().linear_velocity(),
                    angular_velocity: body.state().angular_velocity(),
                })
                .map_err(|cause| StepError::Aerodynamics {
                    body_id,
                    cause: crate::aerodynamics::AerodynamicsError::Hull(cause),
                })?;
            body.apply_force(wrench.force).map_err(|_| error())?;
            body.apply_torque(wrench.moment).map_err(|_| error())?;
        }
        let mut drag = Vec3::ZERO;
        let wings: &[crate::aerodynamics::wing::WingSurface] = match primary {
            crate::aerodynamics::Aerodynamics::Wing(surface) => std::slice::from_ref(surface),
            crate::aerodynamics::Aerodynamics::Wings(surfaces) => surfaces,
            _ => &[],
        };
        let part_wings = cluster
            .into_iter()
            .flat_map(|cluster| cluster.parts())
            .map(|id| {
                (
                    *id,
                    parts
                        .get(id.dense)
                        .filter(|part| id.world == body_id.world && part.body() == body_id)
                        .ok_or_else(error),
                )
            });
        let mut evaluate_wing = |surface: &crate::aerodynamics::wing::WingSurface,
                                 local: crate::transform::Transform,
                                 visibility: Option<crate::aerodynamics::wing::WingVisibility>,
                                 visibility_required: bool,
                                 owner: Option<PartId>|
         -> Result<(), StepError> {
            let transform = body
                .state()
                .transform()
                .compose(&local)
                .map_err(|_| error())?;
            let local_center = local
                .inverse()
                .map_err(|_| error())?
                .transform_point(body.physics().mass().ok_or_else(error)?.center_of_mass());
            let point = transform.transform_point(surface.local_transform().translation());
            if !point.is_finite() {
                return Err(error());
            }
            let atmosphere = atmosphere
                .state_at(point)
                .map_err(|cause| StepError::Aerodynamics { body_id, cause })?;
            if atmosphere.density() != 0.0 {
                if visibility_required && visibility.is_none() {
                    return Err(StepError::Aerodynamics {
                        body_id,
                        cause: crate::aerodynamics::AerodynamicsError::Wing(
                            crate::aerodynamics::wing::WingError::ShieldingNotPrepared,
                        ),
                    });
                }
                let flow = crate::aerodynamics::pressure::PressureFlow {
                    atmosphere,
                    sound_speed: atmosphere.sound_speed().unwrap_or(mesh_sound_speed),
                    transform,
                    center_of_mass: local_center,
                    linear_velocity: body.state().linear_velocity(),
                    angular_velocity: body.state().angular_velocity(),
                };
                let wrench =
                    surface
                        .evaluate_with_visibility(flow, visibility)
                        .map_err(|cause| StepError::Aerodynamics {
                            body_id,
                            cause: crate::aerodynamics::AerodynamicsError::Wing(cause),
                        })?;
                body.apply_force(wrench.force).map_err(|_| error())?;
                body.apply_torque(wrench.moment).map_err(|_| error())?;
                if let (Some(part), Some(wrenches)) = (owner, part_wing_wrenches.as_deref_mut()) {
                    wrenches.try_reserve(1).map_err(|_| error())?;
                    wrenches.push(PartWingWrench {
                        body: body_id,
                        part,
                        wrench,
                    });
                }
            }
            Ok(())
        };
        for surface in wings {
            evaluate_wing(
                surface,
                crate::transform::Transform::identity(),
                None,
                false,
                None,
            )?;
        }
        for (part_id, part) in part_wings {
            let part = part?;
            for (index, surface) in part.aerodynamic_wings.iter().enumerate() {
                let visibility = if surface.has_footprint() {
                    wing_visibility
                        .and_then(|values| {
                            values
                                .binary_search_by_key(&(part_id, index), |value| {
                                    (value.part, value.index)
                                })
                                .ok()
                                .and_then(|i| values.get(i))
                        })
                        .map(|value| value.visibility)
                } else {
                    None
                };
                evaluate_wing(
                    surface,
                    part.local_transform(),
                    visibility,
                    surface.has_footprint() && wing_visibility.is_some(),
                    Some(part_id),
                )?;
            }
        }
        let part_hulls = cluster
            .into_iter()
            .flat_map(|cluster| cluster.parts())
            .map(|id| {
                (
                    *id,
                    parts
                        .get(id.dense)
                        .filter(|part| id.world == body_id.world && part.body() == body_id)
                        .ok_or_else(error),
                )
            });
        for (part_id, part) in part_hulls {
            let part = part?;
            let part_transform = body
                .state()
                .transform()
                .compose(&part.local_transform())
                .map_err(|_| error())?;
            let part_com = part_transform
                .inverse()
                .map_err(|_| error())?
                .transform_point(center);
            let derived = assembly_wave
                .filter(|(owner, _)| *owner == part_id)
                .map(|(_, section)| section);
            for section in part.aerodynamic_hull.iter().chain(derived) {
                let point = part_transform.transform_point(section.local_transform().translation());
                if !point.is_finite() {
                    return Err(error());
                }
                let state = atmosphere
                    .state_at(point)
                    .map_err(|cause| StepError::Aerodynamics { body_id, cause })?;
                let wrench = section
                    .evaluate(crate::aerodynamics::pressure::PressureFlow {
                        atmosphere: state,
                        sound_speed: state.sound_speed().unwrap_or(mesh_sound_speed),
                        transform: part_transform,
                        center_of_mass: part_com,
                        linear_velocity: body.state().linear_velocity(),
                        angular_velocity: body.state().angular_velocity(),
                    })
                    .map_err(|cause| StepError::Aerodynamics {
                        body_id,
                        cause: crate::aerodynamics::AerodynamicsError::Hull(cause),
                    })?;
                body.apply_force(wrench.force).map_err(|_| error())?;
                body.apply_torque(wrench.moment).map_err(|_| error())?;
                if let Some(wrenches) = part_hull_wrenches.as_deref_mut() {
                    wrenches.try_reserve(1).map_err(|_| error())?;
                    wrenches.push(PartHullWrench {
                        body: body_id,
                        part: part_id,
                        wrench,
                    });
                }
            }
        }
        let mut drag_torque = Vec3::ZERO;
        let mut lift_force = Vec3::ZERO;
        let mut lift_torque = Vec3::ZERO;
        for surface in primary_surface.iter().chain(surfaces.iter()) {
            let properties = surface.properties();
            let target_transform = match surface.target() {
                AerodynamicTarget::Body => body.state().transform(),
                AerodynamicTarget::Part(part) => parts
                    .get(part.dense)
                    .filter(|entry| part.world == body_id.world && entry.body() == body_id)
                    .and_then(|entry| {
                        body.state()
                            .transform()
                            .compose(&entry.local_transform())
                            .ok()
                    })
                    .ok_or_else(error)?,
            };
            let point = match surface.target() {
                AerodynamicTarget::Body => center.checked_add(
                    target_transform.transform_vector(properties.local_application_offset()),
                ),
                AerodynamicTarget::Part(_) => {
                    Some(target_transform.transform_point(properties.local_application_offset()))
                }
            }
            .filter(|point| point.is_finite())
            .ok_or_else(error)?;
            let local_atmosphere = atmosphere
                .state_at(point)
                .map_err(|cause| StepError::Aerodynamics { body_id, cause })?;
            if local_atmosphere.density() == 0.0 {
                if let Some(entries) = raw_simple_surfaces.as_deref_mut() {
                    entries.try_reserve(1).map_err(|_| error())?;
                    let zero = crate::aerodynamics::pressure::PressureWrench {
                        force: Vec3::ZERO,
                        moment: Vec3::ZERO,
                    };
                    entries.push(RawSimpleSurfaceWrench {
                        body: body_id,
                        target: surface.target(),
                        drag: zero,
                        lift: zero,
                    });
                }
                continue;
            }
            let offset = point.checked_sub(center).ok_or_else(error)?;
            let point_velocity = body
                .state()
                .angular_velocity()
                .checked_cross(offset)
                .and_then(|velocity| velocity.checked_add(body.state().linear_velocity()))
                .ok_or_else(error)?;
            let point_properties = properties
                .with_local_application_offset(Vec3::ZERO)
                .map_err(|_| error())?;
            let (surface_drag, lift) = point_properties
                .force_components(
                    local_atmosphere,
                    target_transform,
                    point_velocity,
                    Vec3::ZERO,
                )
                .map_err(|cause| StepError::Aerodynamics { body_id, cause })?;
            drag = drag.checked_add(surface_drag).ok_or_else(error)?;
            let surface_drag_torque = offset.checked_cross(surface_drag).ok_or_else(error)?;
            drag_torque = drag_torque
                .checked_add(surface_drag_torque)
                .ok_or_else(error)?;
            lift_force = lift_force.checked_add(lift).ok_or_else(error)?;
            let surface_lift_torque = offset.checked_cross(lift).ok_or_else(error)?;
            lift_torque = lift_torque
                .checked_add(surface_lift_torque)
                .ok_or_else(error)?;
            if let Some(entries) = raw_simple_surfaces.as_deref_mut() {
                entries.try_reserve(1).map_err(|_| error())?;
                entries.push(RawSimpleSurfaceWrench {
                    body: body_id,
                    target: surface.target(),
                    drag: crate::aerodynamics::pressure::PressureWrench {
                        force: surface_drag,
                        moment: surface_drag_torque,
                    },
                    lift: crate::aerodynamics::pressure::PressureWrench {
                        force: lift,
                        moment: surface_lift_torque,
                    },
                });
            }
        }
        let raw_drag = crate::aerodynamics::pressure::PressureWrench {
            force: drag,
            moment: drag_torque,
        };
        let raw_lift = crate::aerodynamics::pressure::PressureWrench {
            force: lift_force,
            moment: lift_torque,
        };
        if drag == Vec3::ZERO
            && drag_torque == Vec3::ZERO
            && lift_force == Vec3::ZERO
            && lift_torque == Vec3::ZERO
        {
            if (!surfaces.is_empty() || primary_surface.is_some())
                && let Some(output) = simple_surface_correction
            {
                *output = Some(SimpleSurfaceCorrection {
                    body: body_id,
                    raw_drag,
                    raw_lift,
                    applied_drag: raw_drag,
                    applied_lift: raw_lift,
                });
            }
            return Ok(());
        }
        let center_wind = atmosphere
            .state_at(center)
            .map_err(|cause| StepError::Aerodynamics { body_id, cause })?
            .wind_velocity();
        let mut scale = 1.0_f64;
        if let Some(dt) = dt {
            let mass = body.physics().mass().ok_or_else(error)?;
            let rotation = body.state().transform().rotation();
            let local_torque = rotation
                .conjugate()
                .rotate_vector(drag_torque)
                .ok_or_else(error)?;
            let local_angular_acceleration = mass
                .inertia()
                .solve_spd(local_torque)
                .map_err(|_| error())?;
            let angular_acceleration = rotation
                .rotate_vector(local_angular_acceleration)
                .ok_or_else(error)?;
            let relative_velocity = body
                .state()
                .linear_velocity()
                .checked_sub(center_wind)
                .ok_or_else(error)?;
            let power = relative_velocity
                .checked_dot(drag)
                .and_then(|linear| {
                    body.state()
                        .angular_velocity()
                        .checked_dot(drag_torque)
                        .map(|angular| linear + angular)
                })
                .ok_or_else(error)?;
            let curvature = drag
                .checked_dot(drag)
                .and_then(|linear| {
                    drag_torque
                        .checked_dot(angular_acceleration)
                        .map(|angular| linear / mass.mass() + angular)
                })
                .ok_or_else(error)?;
            if !power.is_finite() || !curvature.is_finite() {
                return Err(error());
            }
            if curvature > 0.0 {
                scale = (-power / dt / curvature).clamp(0.0, 1.0);
            }
        }
        let applied_drag = crate::aerodynamics::pressure::PressureWrench {
            force: drag.checked_mul_scalar(scale).ok_or_else(error)?,
            moment: drag_torque.checked_mul_scalar(scale).ok_or_else(error)?,
        };
        body.apply_force(applied_drag.force).map_err(|_| error())?;
        body.apply_torque(applied_drag.moment)
            .map_err(|_| error())?;
        if let Some(dt) = dt
            && (lift_force != Vec3::ZERO || lift_torque != Vec3::ZERO)
        {
            let mass = body.physics().mass().ok_or_else(error)?;
            let rotation = body.state().transform().rotation();
            let inverse = rotation.conjugate();
            let drag_angular_acceleration = rotation
                .rotate_vector(
                    mass.inertia()
                        .solve_spd(inverse.rotate_vector(drag_torque).ok_or_else(error)?)
                        .map_err(|_| error())?,
                )
                .ok_or_else(error)?;
            let angular_velocity = body
                .state()
                .angular_velocity()
                .checked_add(
                    drag_angular_acceleration
                        .checked_mul_scalar(scale * dt)
                        .ok_or_else(error)?,
                )
                .ok_or_else(error)?;
            let local_omega = inverse.rotate_vector(angular_velocity).ok_or_else(error)?;
            let angular_momentum = rotation
                .rotate_vector(
                    mass.inertia()
                        .checked_mul_vector(local_omega)
                        .ok_or_else(error)?,
                )
                .ok_or_else(error)?;
            let relative_velocity = body
                .state()
                .linear_velocity()
                .checked_sub(center_wind)
                .and_then(|velocity| {
                    drag.checked_mul_scalar(scale * dt / mass.mass())
                        .and_then(|change| velocity.checked_add(change))
                })
                .ok_or_else(error)?;
            let linear_momentum = relative_velocity
                .checked_mul_scalar(mass.mass())
                .ok_or_else(error)?;
            let metric = relative_velocity
                .checked_dot(linear_momentum)
                .and_then(|linear| {
                    angular_velocity
                        .checked_dot(angular_momentum)
                        .map(|angular| linear + angular)
                })
                .filter(|value| value.is_finite() && *value >= 0.0)
                .ok_or_else(error)?;
            if metric == 0.0 {
                if let Some(output) = simple_surface_correction {
                    *output = Some(SimpleSurfaceCorrection {
                        body: body_id,
                        raw_drag,
                        raw_lift,
                        applied_drag,
                        applied_lift: crate::aerodynamics::pressure::PressureWrench {
                            force: Vec3::ZERO,
                            moment: Vec3::ZERO,
                        },
                    });
                }
                return Ok(());
            }
            let power = relative_velocity
                .checked_dot(lift_force)
                .and_then(|linear| {
                    angular_velocity
                        .checked_dot(lift_torque)
                        .map(|angular| linear + angular)
                })
                .filter(|value| value.is_finite())
                .ok_or_else(error)?;
            let projection = power / metric;
            lift_force = lift_force
                .checked_sub(
                    linear_momentum
                        .checked_mul_scalar(projection)
                        .ok_or_else(error)?,
                )
                .ok_or_else(error)?;
            lift_torque = lift_torque
                .checked_sub(
                    angular_momentum
                        .checked_mul_scalar(projection)
                        .ok_or_else(error)?,
                )
                .ok_or_else(error)?;
            let local_lift_torque = inverse.rotate_vector(lift_torque).ok_or_else(error)?;
            let angular_acceleration = rotation
                .rotate_vector(
                    mass.inertia()
                        .solve_spd(local_lift_torque)
                        .map_err(|_| error())?,
                )
                .ok_or_else(error)?;
            let curvature = lift_force
                .checked_dot(lift_force)
                .and_then(|linear| {
                    lift_torque
                        .checked_dot(angular_acceleration)
                        .map(|angular| linear / mass.mass() + angular)
                })
                .filter(|value| value.is_finite() && *value >= 0.0)
                .ok_or_else(error)?;
            let rate = (curvature / metric).sqrt();
            let half_angle = 0.5 * dt * rate;
            if !half_angle.is_finite() {
                return Err(error());
            }
            let (factor, damping) = if half_angle <= 1.0 {
                let factor = 1.0 / (1.0 + half_angle * half_angle);
                (factor, rate * half_angle * factor)
            } else {
                let inverse_angle = 1.0 / half_angle;
                let denominator = 1.0 + inverse_angle * inverse_angle;
                (
                    inverse_angle * inverse_angle / denominator,
                    rate * inverse_angle / denominator,
                )
            };
            lift_force = lift_force
                .checked_mul_scalar(factor)
                .and_then(|force| {
                    linear_momentum
                        .checked_mul_scalar(damping)
                        .and_then(|correction| force.checked_sub(correction))
                })
                .ok_or_else(error)?;
            lift_torque = lift_torque
                .checked_mul_scalar(factor)
                .and_then(|torque| {
                    angular_momentum
                        .checked_mul_scalar(damping)
                        .and_then(|correction| torque.checked_sub(correction))
                })
                .ok_or_else(error)?;
        }
        body.apply_force(lift_force).map_err(|_| error())?;
        body.apply_torque(lift_torque).map_err(|_| error())?;
        if let Some(output) = simple_surface_correction {
            *output = Some(SimpleSurfaceCorrection {
                body: body_id,
                raw_drag,
                raw_lift,
                applied_drag,
                applied_lift: crate::aerodynamics::pressure::PressureWrench {
                    force: lift_force,
                    moment: lift_torque,
                },
            });
        }
        Ok(())
    }

    fn prepare_voxel_wave(
        model: &crate::aerodynamics::wave::VoxelWave,
        body_id: BodyId,
        body: &BodyMotion,
        atmosphere: Atmosphere,
        cache: &mut crate::aerodynamics::cache::PreparedGeometryCache,
    ) -> Result<Option<crate::aerodynamics::hull::HullSection>, StepError> {
        if body.physics().body_type() != BodyType::Dynamic {
            return Ok(None);
        }
        let point = body
            .state()
            .transform()
            .transform_point(model.section.local_transform().translation());
        let state = atmosphere
            .state_at(point)
            .map_err(|cause| StepError::Aerodynamics { body_id, cause })?;
        if state.density() == 0.0 {
            return Ok(None);
        }
        cache
            .prepare(
                crate::aerodynamics::cache::GeometryCacheKey {
                    body: body_id,
                    geometry_version: 0,
                    config_version: 1,
                },
                &model.mesh,
                model.geometry_config,
            )
            .map_err(|cause| StepError::Aerodynamics {
                body_id,
                cause: crate::aerodynamics::AerodynamicsError::Geometry(cause),
            })?;
        let section = cache
            .configure_voxel_wave_drag(model.section.clone(), model.axis, model.smoothing_radius)
            .map_err(|cause| StepError::Aerodynamics {
                body_id,
                cause: crate::aerodynamics::AerodynamicsError::Hull(cause),
            })?;
        Ok(Some(section))
    }

    fn prepare_aerodynamics_updates<E: StepExecutor>(
        &mut self,
        executor: &E,
        workers: usize,
        dt: f64,
    ) -> Result<(), StepError> {
        self.recycle_aerodynamics_updates();
        self.part_hull_pending.clear();
        self.part_wing_pending.clear();
        self.simple_surface_pending.clear();
        self.raw_simple_pending.clear();
        let atmosphere = self.atmosphere;
        if matches!(atmosphere, Atmosphere::Vacuum) {
            return Ok(());
        };

        for (dense_id, entry) in self.bodies.iter_mut() {
            if (entry.body.aerodynamic_surfaces.is_empty()
                && entry.body.base_pressure_patches.is_empty()
                && !assembly_aerodynamics::wings_configured(entry, &self.parts)
                && !assembly_aerodynamics::hull_configured(entry, &self.parts)
                && matches!(
                    entry.body.aerodynamics(),
                    crate::aerodynamics::Aerodynamics::None
                )
                && !assembly_aerodynamics::configured(
                    entry,
                    self.part_aerodynamic_geometry.as_ref(),
                ))
                || entry.body.physics().body_type() != BodyType::Dynamic
            {
                continue;
            }
            let body_id = BodyId {
                world: self.world_id,
                dense: dense_id,
            };
            if self.aerodynamics_body_updates.try_reserve(1).is_err() {
                return Err(StepError::Aerodynamics {
                    body_id,
                    cause: crate::aerodynamics::AerodynamicsError::ArithmeticOverflow,
                });
            }
            if entry.mesh_aerodynamics.as_ref().is_some_and(|prepared| {
                match (
                    &prepared.identity,
                    &entry.body.aerodynamic_geometry_identity,
                ) {
                    (Some(cached), Some(current)) => !std::sync::Arc::ptr_eq(cached, current),
                    (None, None) => false,
                    _ => true,
                }
            }) {
                entry.mesh_aerodynamics = None;
                entry.collider_aerodynamic_mesh = None;
            }
            let voxel_wave = if let crate::aerodynamics::Aerodynamics::VoxelWave(model) =
                entry.body.aerodynamics()
            {
                let prepared = entry
                    .mesh_aerodynamics
                    .get_or_insert_with(|| MeshAerodynamics {
                        identity: entry.body.aerodynamic_geometry_identity.clone(),
                        cache: crate::aerodynamics::cache::PreparedGeometryCache::default(),
                        config: model.geometry_config,
                        sound_speed: self.mesh_sound_speed,
                    });
                Self::prepare_voxel_wave(
                    model,
                    body_id,
                    &entry.body.motion(),
                    atmosphere,
                    &mut prepared.cache,
                )?
            } else {
                None
            };
            let assembly_prepared = assembly_aerodynamics::prepare(
                body_id,
                entry,
                &self.parts,
                self.part_aerodynamic_geometry.as_ref(),
                &self.colliders,
                &mut self.assembly_aerodynamics,
                atmosphere,
            )?;
            let (mut mesh_patches, assembly_wave, wing_visibility) = match assembly_prepared {
                Some(prepared) => (
                    prepared.patches,
                    prepared.wave,
                    Some(prepared.wing_visibility),
                ),
                None => (None, None, None),
            };
            let mut sphere_patches = None;
            // Only the primary geometry samples the COM. Additional surfaces
            // still evaluate their own positions, including when the COM is in vacuum.
            let primary_atmosphere = if matches!(
                entry.body.aerodynamics(),
                crate::aerodynamics::Aerodynamics::Mesh(_)
                    | crate::aerodynamics::Aerodynamics::Colliders
            ) {
                let center = entry
                    .body
                    .world_center_of_mass()
                    .map_err(|_| StepError::Aerodynamics {
                        body_id,
                        cause: crate::aerodynamics::AerodynamicsError::ArithmeticOverflow,
                    })?
                    .ok_or(StepError::Aerodynamics {
                        body_id,
                        cause: crate::aerodynamics::AerodynamicsError::ArithmeticOverflow,
                    })?;
                Some(
                    atmosphere
                        .state_at(center)
                        .map_err(|cause| StepError::Aerodynamics { body_id, cause })?,
                )
                .filter(|state| state.density() != 0.0)
            } else {
                None
            };
            if primary_atmosphere.is_some()
                && matches!(
                    entry.body.aerodynamics(),
                    crate::aerodynamics::Aerodynamics::Colliders
                )
                && entry.collider_aerodynamic_mesh.is_none()
            {
                let mut next = entry.first_collider;
                let attached = std::iter::from_fn(|| {
                    let id = next?;
                    let collider = self.colliders.get(id.dense)?;
                    next = collider.next;
                    Some(&collider.collider)
                });
                entry.collider_aerodynamic_mesh = Some(
                    crate::aerodynamics::colliders::collider_mesh(
                        attached,
                        entry.collider_tessellation,
                    )
                    .map_err(|cause| StepError::Aerodynamics { body_id, cause })?,
                );
            }
            let mesh = match entry.body.aerodynamics() {
                crate::aerodynamics::Aerodynamics::Mesh(mesh) => Some(mesh),
                crate::aerodynamics::Aerodynamics::Colliders => {
                    entry.collider_aerodynamic_mesh.as_ref()
                }
                _ => None,
            };
            if let (Some(mesh), Some(primary_atmosphere)) = (mesh, primary_atmosphere) {
                if let Some((center, radius)) = entry.body.potential_sphere {
                    if entry.cluster.is_some()
                        || !entry.body.base_pressure_patches.is_empty()
                        || mesh_patches.is_some()
                        || assembly_wave.is_some()
                    {
                        return Err(StepError::Aerodynamics {
                            body_id,
                            cause: crate::aerodynamics::AerodynamicsError::UnsupportedModel,
                        });
                    }
                    // A strict work budget limits the quadratic convexity check.
                    const SPHERE_MESH_WORK_LIMIT: usize = 100_000_000;
                    // Roundoff tolerance for tessellated vertices, relative to radius.
                    const SPHERE_MESH_RELATIVE_TOLERANCE: f64 = 1e-10;
                    sphere_patches = Some(
                        crate::aerodynamics::pressure::spherical_mesh_pressure_patches(
                            mesh,
                            center,
                            radius,
                            SPHERE_MESH_RELATIVE_TOLERANCE,
                            SPHERE_MESH_WORK_LIMIT,
                        )
                        .map_err(|cause| StepError::Aerodynamics {
                            body_id,
                            cause: crate::aerodynamics::AerodynamicsError::Pressure(cause),
                        })?,
                    );
                } else {
                    if !entry.body.base_pressure_patches.is_empty()
                        && entry.body.base_pressure_triangles.is_none()
                    {
                        return Err(StepError::Aerodynamics {
                            body_id,
                            cause: crate::aerodynamics::AerodynamicsError::UnsupportedModel,
                        });
                    }
                    let config = entry
                        .mesh_aerodynamics
                        .as_ref()
                        .map(|prepared| prepared.config)
                        .unwrap_or(crate::aerodynamics::geometry::GeometryConfig {
                            cell_size: 0.125,
                            max_cells: 1_048_576,
                            max_work: 100_000_000,
                            open_mesh_policy:
                                crate::aerodynamics::geometry::OpenMeshPolicy::SurfaceOnly,
                            weld_tolerance: matches!(
                                entry.body.aerodynamics(),
                                crate::aerodynamics::Aerodynamics::Colliders
                            )
                            .then_some(1e-10),
                            max_patch_work: 100_000_000,
                        });
                    let prepared =
                        entry
                            .mesh_aerodynamics
                            .get_or_insert_with(|| MeshAerodynamics {
                                identity: entry.body.aerodynamic_geometry_identity.clone(),
                                cache: crate::aerodynamics::cache::PreparedGeometryCache::default(),
                                config,
                                sound_speed: self.mesh_sound_speed,
                            });
                    prepared.config = config;
                    prepared.sound_speed = self.mesh_sound_speed;
                    let key = crate::aerodynamics::cache::GeometryCacheKey {
                        body: body_id,
                        geometry_version: 0,
                        config_version: 1,
                    };
                    let geometry = prepared.cache.prepare(key, mesh, config).map_err(|cause| {
                        StepError::Aerodynamics {
                            body_id,
                            cause: crate::aerodynamics::AerodynamicsError::Geometry(cause),
                        }
                    })?;
                    let relative = entry
                        .body
                        .state()
                        .linear_velocity()
                        .checked_sub(primary_atmosphere.wind_velocity())
                        .ok_or(StepError::Aerodynamics {
                            body_id,
                            cause: crate::aerodynamics::AerodynamicsError::ArithmeticOverflow,
                        })?;
                    let direction = entry
                        .body
                        .state()
                        .transform()
                        .rotation()
                        .conjugate()
                        .rotate_vector(relative)
                        .ok_or(StepError::Aerodynamics {
                            body_id,
                            cause: crate::aerodynamics::AerodynamicsError::ArithmeticOverflow,
                        })?;
                    let inverse = entry.body.state().transform().inverse().map_err(|_| {
                        StepError::Aerodynamics {
                            body_id,
                            cause: crate::aerodynamics::AerodynamicsError::ArithmeticOverflow,
                        }
                    })?;
                    let omega = inverse.transform_vector(entry.body.state().angular_velocity());
                    let center = entry
                        .body
                        .physics()
                        .mass()
                        .ok_or(StepError::Aerodynamics {
                            body_id,
                            cause: crate::aerodynamics::AerodynamicsError::ArithmeticOverflow,
                        })?
                        .center_of_mass();
                    if let Some(selected) = entry
                        .body
                        .base_pressure_triangles
                        .as_deref()
                        .filter(|s| !s.is_empty())
                    {
                        crate::aerodynamics::pressure::verify_unique_base_sources(
                            selected,
                            mesh,
                            config.max_patch_work,
                        )
                        .map_err(|cause| StepError::Aerodynamics { body_id, cause })?;
                        let visible = geometry
                            .surface_patches_with_flow(
                                &[crate::aerodynamics::geometry::GeometryInstance {
                                    mesh,
                                    local_transform: crate::transform::Transform::identity(),
                                    owner: crate::aerodynamics::geometry::GeometryOwner::Body,
                                }],
                                config.max_patch_work,
                                |point| {
                                    point.checked_sub(center)
                            .and_then(|arm| omega.checked_cross(arm))
                            .and_then(|velocity| direction.checked_add(velocity))
                            .and_then(|velocity| Vec3::ZERO.checked_sub(velocity))
                            .ok_or(crate::aerodynamics::geometry::GeometryError::ArithmeticOverflow)
                                },
                            )
                            .map_err(|cause| StepError::Aerodynamics {
                                body_id,
                                cause: crate::aerodynamics::AerodynamicsError::Geometry(cause),
                            })?;
                        crate::aerodynamics::pressure::verify_full_base_visibility(
                            selected, &visible,
                        )
                        .map_err(|cause| StepError::Aerodynamics {
                            body_id,
                            cause: crate::aerodynamics::AerodynamicsError::Pressure(cause),
                        })?;
                    }
                    mesh_patches = Some(
                    geometry
                        .surface_patches_with_flow(
                            &[crate::aerodynamics::geometry::GeometryInstance {
                                mesh,
                                local_transform: crate::transform::Transform::identity(),
                                owner: crate::aerodynamics::geometry::GeometryOwner::Body,
                            }],
                            config.max_patch_work,
                            |point| point.checked_sub(center)
                                .and_then(|arm| omega.checked_cross(arm))
                                .and_then(|velocity| direction.checked_add(velocity))
                                .ok_or(crate::aerodynamics::geometry::GeometryError::ArithmeticOverflow),
                        )
                        .map_err(|cause| StepError::Aerodynamics {
                            body_id,
                            cause: crate::aerodynamics::AerodynamicsError::Geometry(cause),
                        })?,
                );
                }
            }
            self.aerodynamics_body_updates.push(AerodynamicsBodyUpdate {
                id: body_id,
                error: None,
                original: entry.body.motion(),
                prepared: entry.body.motion(),
                part_hull_wrenches: self.part_hull_buffers.pop().unwrap_or_default(),
                part_wing_wrenches: self.part_wing_buffers.pop().unwrap_or_default(),
                simple_surface_correction: None,
                raw_simple_surfaces: self.raw_simple_buffers.pop().unwrap_or_default(),
                applied_mesh_pressure: None,
                loads: PreparedAerodynamicLoads {
                    mesh_patches,
                    sphere_patches,
                    voxel_wave,
                    assembly_wave,
                    wing_visibility,
                },
            });
        }

        let bodies = &self.bodies;
        let chunk_size = self
            .aerodynamics_body_updates
            .len()
            .div_ceil(workers.max(1))
            .max(1);
        executor
            .for_each_chunk_mut(
                &mut self.aerodynamics_body_updates,
                chunk_size,
                |_, updates| {
                    for update in updates {
                        update.error = bodies
                            .get(update.id.dense)
                            .map(|entry| {
                                Self::apply_aerodynamics(
                                    atmosphere,
                                    entry.body.aerodynamics(),
                                    &entry.body.aerodynamic_surfaces,
                                    update.id,
                                    &mut update.prepared,
                                    &self.parts,
                                    AerodynamicsInputs::new(self.mesh_sound_speed)
                                        .with_base_pressure(&entry.body.base_pressure_patches)
                                        .with_cluster(entry.cluster.as_ref())
                                        .with_timestep(Some(dt))
                                        .with_loads(&update.loads)
                                        .with_part_hull_wrenches(&mut update.part_hull_wrenches)
                                        .with_part_wing_wrenches(&mut update.part_wing_wrenches)
                                        .with_simple_surface_correction(
                                            &mut update.simple_surface_correction,
                                        )
                                        .with_raw_simple_surfaces(&mut update.raw_simple_surfaces)
                                        .with_applied_mesh_pressure(
                                            &mut update.applied_mesh_pressure,
                                        ),
                                )
                            })
                            .unwrap_or(Ok(()))
                            .err();
                    }
                },
            )
            .map_err(|_| StepError::Collision(CollisionError::WorkerFailed))?;
        if let Some(error) = self
            .aerodynamics_body_updates
            .iter()
            .find_map(|update| update.error)
        {
            return Err(error);
        }

        for update in &self.aerodynamics_body_updates {
            self.part_hull_pending
                .try_reserve(update.part_hull_wrenches.len())
                .map_err(|_| StepError::Aerodynamics {
                    body_id: update.id,
                    cause: crate::aerodynamics::AerodynamicsError::ArithmeticOverflow,
                })?;
            self.part_hull_pending
                .extend_from_slice(&update.part_hull_wrenches);
            self.part_wing_pending
                .try_reserve(update.part_wing_wrenches.len())
                .map_err(|_| StepError::Aerodynamics {
                    body_id: update.id,
                    cause: crate::aerodynamics::AerodynamicsError::ArithmeticOverflow,
                })?;
            self.part_wing_pending
                .extend_from_slice(&update.part_wing_wrenches);
            if let Some(correction) = update.simple_surface_correction {
                self.simple_surface_pending.try_reserve(1).map_err(|_| {
                    StepError::Aerodynamics {
                        body_id: update.id,
                        cause: crate::aerodynamics::AerodynamicsError::ArithmeticOverflow,
                    }
                })?;
                self.simple_surface_pending.push(correction);
            }
            self.raw_simple_pending
                .try_reserve(update.raw_simple_surfaces.len())
                .map_err(|_| StepError::Aerodynamics {
                    body_id: update.id,
                    cause: crate::aerodynamics::AerodynamicsError::ArithmeticOverflow,
                })?;
            self.raw_simple_pending
                .extend_from_slice(&update.raw_simple_surfaces);
        }

        // This diagnostic pass never applies forces; the solver path above is
        // unchanged. The workspace is committed only after the entire step.
        for update in &self.aerodynamics_body_updates {
            if update.loads.sphere_patches.is_some() {
                continue;
            }
            let Some(patches) = update.loads.mesh_patches.as_ref() else {
                continue;
            };
            let error = || StepError::Aerodynamics {
                body_id: update.id,
                cause: crate::aerodynamics::AerodynamicsError::ArithmeticOverflow,
            };
            let center = update
                .original
                .world_center_of_mass()
                .map_err(|_| error())?
                .ok_or_else(error)?;
            let state = atmosphere
                .state_at(center)
                .map_err(|cause| StepError::Aerodynamics {
                    body_id: update.id,
                    cause,
                })?;
            if state.density() == 0.0 {
                continue;
            }
            let transform = update.original.state().transform();
            let flow = crate::aerodynamics::pressure::PressureFlow {
                atmosphere: state,
                sound_speed: state.sound_speed().unwrap_or(self.mesh_sound_speed),
                transform,
                center_of_mass: transform
                    .inverse()
                    .map_err(|_| error())?
                    .transform_point(center),
                linear_velocity: update.original.state().linear_velocity(),
                angular_velocity: update.original.state().angular_velocity(),
            };
            self.owner_pressure_owners.clear();
            self.owner_pressure_owners
                .try_reserve(patches.iter().count())
                .map_err(|_| error())?;
            self.owner_pressure_owners
                .extend(patches.iter().map(|patch| patch.owner));
            self.owner_pressure_owners.sort_unstable();
            self.owner_pressure_owners.dedup();
            let map = self.owner_pressure.get_or_insert_with(HashMap::new);
            map.try_reserve(1).map_err(|_| error())?;
            if !map.contains_key(&update.id) {
                let workspace = crate::aerodynamics::pressure::OwnerPressureWorkspace::new(&[])
                    .ok_or_else(error)?;
                map.insert(update.id, workspace);
            }
            let workspace = map.get_mut(&update.id).ok_or_else(error)?;
            workspace
                .configure_pending(&self.owner_pressure_owners)
                .and_then(|()| workspace.prepare(patches, flow))
                .map_err(|cause| StepError::Aerodynamics {
                    body_id: update.id,
                    cause: crate::aerodynamics::AerodynamicsError::Pressure(cause),
                })?;
            if workspace
                .staged_total()
                .map_err(|cause| StepError::Aerodynamics {
                    body_id: update.id,
                    cause: crate::aerodynamics::AerodynamicsError::Pressure(cause),
                })?
                != update.applied_mesh_pressure
            {
                return Err(StepError::InvalidStepResults);
            }
        }

        for update in &self.aerodynamics_body_updates {
            if let Some(entry) = self
                .bodies
                .get_mut(update.id.dense)
                .filter(|_| update.id.world == self.world_id)
            {
                entry.body.set_motion(update.prepared);
            }
        }
        Ok(())
    }

    fn rollback_aerodynamics_updates(&mut self) {
        if let Some(workspaces) = &mut self.owner_pressure {
            for workspace in workspaces.values_mut() {
                workspace.rollback();
            }
        }
        for update in &self.aerodynamics_body_updates {
            if let Some(entry) = self
                .bodies
                .get_mut(update.id.dense)
                .filter(|_| update.id.world == self.world_id)
            {
                entry.body.set_motion(update.original);
            }
        }
        self.recycle_aerodynamics_updates();
    }

    fn recycle_aerodynamics_updates(&mut self) {
        for mut update in self.aerodynamics_body_updates.drain(..) {
            update.part_hull_wrenches.clear();
            update.part_wing_wrenches.clear();
            update.raw_simple_surfaces.clear();
            // If the pool cannot grow, dropping scratch buffers is safe.
            if self.part_hull_buffers.try_reserve(1).is_ok() {
                self.part_hull_buffers.push(update.part_hull_wrenches);
            }
            if self.part_wing_buffers.try_reserve(1).is_ok() {
                self.part_wing_buffers.push(update.part_wing_wrenches);
            }
            if self.raw_simple_buffers.try_reserve(1).is_ok() {
                self.raw_simple_buffers.push(update.raw_simple_surfaces);
            }
        }
    }

    fn gravity_force_for_body(
        gravity: Option<CentralGravity>,
        fields: &DensePool<Gravity>,
        body_id: BodyId,
        body: &BodyMotion,
    ) -> Result<Vec3, StepError> {
        if gravity.is_none() && fields.is_empty() {
            return Ok(Vec3::ZERO);
        }
        if body.physics().body_type() != BodyType::Dynamic {
            return Ok(Vec3::ZERO);
        }
        let center = body
            .world_center_of_mass()
            .map_err(|_| StepError::Gravity {
                body_id,
                cause: GravityError::ArithmeticOverflow,
            })?
            .ok_or(StepError::Gravity {
                body_id,
                cause: GravityError::ArithmeticOverflow,
            })?;
        let mut acceleration = match gravity {
            Some(gravity) => gravity
                .acceleration_at(center)
                .map_err(|cause| StepError::Gravity { body_id, cause })?,
            None => Vec3::ZERO,
        };
        for (_, field) in fields.iter() {
            let next = field
                .acceleration_at(center)
                .map_err(|cause| StepError::Gravity { body_id, cause })?;
            acceleration = acceleration.checked_add(next).ok_or(StepError::Gravity {
                body_id,
                cause: GravityError::ArithmeticOverflow,
            })?;
        }
        let mass = body.physics().mass().ok_or(StepError::Gravity {
            body_id,
            cause: GravityError::ArithmeticOverflow,
        })?;
        acceleration
            .checked_mul_scalar(mass.mass())
            .ok_or(StepError::Gravity {
                body_id,
                cause: GravityError::ArithmeticOverflow,
            })
    }

    fn apply_gravity(
        gravity: Option<CentralGravity>,
        fields: &DensePool<Gravity>,
        body_id: BodyId,
        body: &mut BodyMotion,
    ) -> Result<(), StepError> {
        let force = Self::gravity_force_for_body(gravity, fields, body_id, body)?;
        if force != Vec3::ZERO {
            body.apply_force(force).map_err(|_| StepError::Gravity {
                body_id,
                cause: GravityError::ArithmeticOverflow,
            })?;
        }
        Ok(())
    }

    fn prepare_gravity_updates<E: StepExecutor>(
        &mut self,
        executor: &E,
        workers: usize,
    ) -> Result<(), StepError> {
        self.gravity_body_updates.clear();
        self.part_gravity_pending.clear();
        if self.central_gravity.is_none() && self.gravity_fields.is_empty() {
            return Ok(());
        }

        for (dense_id, entry) in self.bodies.iter() {
            let body_id = BodyId {
                world: self.world_id,
                dense: dense_id,
            };
            if entry.body.physics().body_type() != BodyType::Dynamic {
                continue;
            }
            if self.gravity_body_updates.try_reserve(1).is_err() {
                self.gravity_body_updates.clear();
                return Err(StepError::Gravity {
                    body_id,
                    cause: GravityError::StorageCapacityExceeded,
                });
            }
            self.gravity_body_updates.push(GravityBodyUpdate {
                id: body_id,
                error: None,
                original: entry.body.motion(),
                prepared: entry.body.motion(),
            });
        }

        let gravity = self.central_gravity;
        let fields = &self.gravity_fields;
        let chunk_size = self
            .gravity_body_updates
            .len()
            .div_ceil(workers.max(1))
            .max(1);
        executor
            .for_each_chunk_mut(&mut self.gravity_body_updates, chunk_size, |_, updates| {
                for update in updates {
                    update.error =
                        Self::apply_gravity(gravity, fields, update.id, &mut update.prepared).err();
                }
            })
            .map_err(|_| StepError::Collision(CollisionError::WorkerFailed))?;
        if let Some(error) = self
            .gravity_body_updates
            .iter()
            .find_map(|update| update.error)
        {
            self.gravity_body_updates.clear();
            return Err(error);
        }

        for update in &self.gravity_body_updates {
            let Some(entry) = self.bodies.get(update.id.dense) else {
                continue;
            };
            let Some(cluster) = entry.cluster.as_ref() else {
                continue;
            };
            let error = || StepError::Gravity {
                body_id: update.id,
                cause: GravityError::ArithmeticOverflow,
            };
            let transform = update.original.state().transform();
            let center = update
                .original
                .world_center_of_mass()
                .map_err(|_| error())?
                .ok_or_else(error)?;
            let mut acceleration = match gravity {
                Some(field) => {
                    field
                        .acceleration_at(center)
                        .map_err(|cause| StepError::Gravity {
                            body_id: update.id,
                            cause,
                        })?
                }
                None => Vec3::ZERO,
            };
            for (_, field) in fields.iter() {
                acceleration = acceleration
                    .checked_add(field.acceleration_at(center).map_err(|cause| {
                        StepError::Gravity {
                            body_id: update.id,
                            cause,
                        }
                    })?)
                    .ok_or_else(error)?;
            }
            self.part_gravity_pending
                .try_reserve(cluster.parts().len())
                .map_err(|_| StepError::Gravity {
                    body_id: update.id,
                    cause: GravityError::StorageCapacityExceeded,
                })?;
            for &part_id in cluster.parts() {
                let part = self
                    .parts
                    .get(part_id.dense)
                    .filter(|part| part_id.world == self.world_id && part.body() == update.id)
                    .ok_or_else(error)?;
                let part_transform = transform
                    .compose(&part.local_transform())
                    .map_err(|_| error())?;
                let part_center = part_transform.transform_point(part.mass().center_of_mass());
                let force = acceleration
                    .checked_mul_scalar(part.mass().mass())
                    .ok_or_else(error)?;
                let moment = part_center
                    .checked_sub(center)
                    .and_then(|arm| arm.checked_cross(force))
                    .ok_or_else(error)?;
                self.part_gravity_pending.push(PartGravityWrench {
                    body: update.id,
                    part: part_id,
                    force,
                    moment,
                });
            }
        }

        for update in &self.gravity_body_updates {
            if let Some(entry) = self
                .bodies
                .get_mut(update.id.dense)
                .filter(|_| update.id.world == self.world_id)
            {
                entry.body.set_motion(update.prepared);
            }
        }
        Ok(())
    }

    fn rollback_gravity_updates(&mut self) {
        for update in &self.gravity_body_updates {
            if let Some(entry) = self
                .bodies
                .get_mut(update.id.dense)
                .filter(|_| update.id.world == self.world_id)
            {
                entry.body.set_motion(update.original);
            }
        }
        self.clear_gravity_updates();
        self.part_gravity_pending.clear();
    }

    fn clear_gravity_updates(&mut self) {
        self.gravity_body_updates.clear();
    }

    fn rollback_force_source_updates(&mut self) {
        for update in &self.force_source_body_updates {
            if let Some(entry) = self
                .bodies
                .get_mut(update.id.dense)
                .filter(|_| update.id.world == self.world_id)
            {
                entry.body.set_motion(update.original);
            }
        }
        self.clear_force_source_updates();
    }

    fn stage_spatial_proxies(&mut self, dt: f64) -> Result<(), CollisionError> {
        for ((body_dense, body_entry), (predicted_id, predicted_state)) in self.bodies.iter().zip(
            self.collision_system
                .context
                .predicted_states
                .iter()
                .copied(),
        ) {
            let body_id = BodyId {
                world: self.world_id,
                dense: body_dense,
            };
            if predicted_id != body_id {
                return Err(CollisionError::InvalidContact);
            }
            let body_type = body_entry.body.physics().body_type();
            let mut next = body_entry.first_collider;
            while let Some(collider_id) = next {
                if collider_id.world != self.world_id {
                    return Err(CollisionError::InvalidContact);
                }
                let collider_entry = self
                    .colliders
                    .get(collider_id.dense)
                    .ok_or(CollisionError::InvalidContact)?;
                if collider_entry.body != body_id {
                    return Err(CollisionError::InvalidContact);
                }
                let bounds = if body_type != BodyType::Static {
                    let trajectory = CcdTrajectory::new(
                        body_entry.body.motion(),
                        *body_entry.body.state(),
                        0.0,
                        0.0,
                        dt,
                        collider_entry.collider.local_transform(),
                        false,
                    );
                    swept_bounds_for_trajectory(
                        &collider_entry.collider,
                        &trajectory,
                        predicted_state,
                    )?
                } else {
                    collider_entry
                        .collider
                        .world_bounds(&predicted_state.transform())
                        .map_err(|_| CollisionError::GeometryBounds)?
                };
                self.collision_system
                    .context
                    .colliders
                    .try_reserve(1)
                    .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
                self.collision_system
                    .context
                    .colliders
                    .push(crate::collision::CollisionProxy {
                        collider: collider_id,
                        body: body_id,
                        bounds,
                        body_type,
                        mode: collider_entry.collider.mode(),
                        filter: collider_entry.collider.filter(),
                        convex: collider_entry.collider.shape().is_convex(),
                    });
                if let Some(bounds) = bounds {
                    self.collision_system
                        .context
                        .spatial_proxies
                        .try_reserve(1)
                        .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
                    self.collision_system
                        .context
                        .spatial_proxies
                        .push((collider_id, bounds));
                }
                next = collider_entry.next;
            }
        }
        Ok(())
    }

    fn commit_predicted_states(&mut self) {
        {
            let world_id = self.world_id;
            let colliders = &self.colliders;
            let query_index = &mut self.query_index;
            let query_index_dirty = &mut self.query_index_dirty;
            for ((dense_id, entry), (body_id, state)) in self.bodies.iter_mut().zip(
                self.collision_system
                    .context
                    .predicted_states
                    .iter()
                    .copied(),
            ) {
                let current_id = BodyId {
                    world: world_id,
                    dense: dense_id,
                };
                debug_assert_eq!(current_id, body_id);
                let transform_changed = entry.body.state().transform() != state.transform();
                entry.body.commit_integrated_state(state);
                if transform_changed
                    && !*query_index_dirty
                    && let Some(index) = query_index.as_mut()
                    && update_query_index_for_body(index, colliders, world_id, body_id, entry)
                        .is_err()
                {
                    *query_index_dirty = true;
                    index.mark_dirty();
                }
            }
        }
        if self.query_index_dirty {
            self.rebuild_query_index();
        }
    }

    fn prepare_collision_candidates(
        &mut self,
        dt: f64,
        worker_count: usize,
        executor: &impl StepExecutor,
    ) -> Result<(), CollisionError> {
        #[cfg(feature = "bench-internals")]
        let started = Instant::now();
        self.stage_spatial_proxies(dt)?;
        self.collision_system.synchronize_spatial_index()?;
        #[cfg(feature = "bench-internals")]
        self.collision_system
            .record_spatial_index(started.elapsed());

        #[cfg(feature = "bench-internals")]
        let started = Instant::now();
        self.collision_system.generate_candidates()?;
        #[cfg(feature = "bench-internals")]
        self.collision_system.record_broad_phase(started.elapsed());

        #[cfg(feature = "bench-internals")]
        let started = Instant::now();
        self.process_continuous_collisions(dt, worker_count, executor)?;
        #[cfg(feature = "bench-internals")]
        self.collision_system.record_ccd(started.elapsed());
        Ok(())
    }

    fn prepare_joint_collision_pairs(&mut self) -> Result<(), CollisionError> {
        let pairs = &mut self.collision_system.context.joint_collision_pairs;
        pairs.clear();
        pairs
            .try_reserve(self.joints.len())
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        for (_, entry) in self.joints.iter() {
            if entry.body_a == entry.body_b
                || !entry.joint.common().enabled
                || entry.joint.common().collide_connected
            {
                continue;
            }
            let pair = if entry.body_a < entry.body_b {
                (entry.body_a, entry.body_b)
            } else {
                (entry.body_b, entry.body_a)
            };
            pairs.push(pair);
        }
        pairs.sort_unstable();
        pairs.dedup();
        Ok(())
    }

    fn prepare_joint_constraints(&mut self) -> Result<(), CollisionError> {
        self.collision_system.context.joint_constraints.clear();
        self.collision_system
            .context
            .joint_constraints
            .try_reserve(self.joints.len())
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        let world_id = self.world_id;
        let bodies = &self.bodies;
        let joints = &self.joints;
        let context = &mut self.collision_system.context;
        for (dense, entry) in joints.iter() {
            let (body_a_id, body_b_id, joint) = (entry.body_a, entry.body_b, entry.joint);
            if !joint.common().enabled {
                continue;
            }
            if body_a_id == body_b_id {
                continue;
            }
            let body_type_a = bodies
                .get(body_a_id.dense)
                .filter(|_| body_a_id.world == world_id)
                .map(|body| body.body.physics().body_type())
                .ok_or(CollisionError::InvalidContact)?;
            let body_type_b = bodies
                .get(body_b_id.dense)
                .filter(|_| body_b_id.world == world_id)
                .map(|body| body.body.physics().body_type())
                .ok_or(CollisionError::InvalidContact)?;
            if body_type_a != BodyType::Dynamic && body_type_b != BodyType::Dynamic {
                continue;
            }
            let prediction_a =
                Self::predicted_body_index(&context.body_indices, world_id, body_a_id)?;
            let prediction_b =
                Self::predicted_body_index(&context.body_indices, world_id, body_b_id)?;
            let body_a = Self::solver_body_index_in_context(
                bodies,
                world_id,
                context,
                body_a_id,
                prediction_a,
            )?;
            let body_b = Self::solver_body_index_in_context(
                bodies,
                world_id,
                context,
                body_b_id,
                prediction_b,
            )?;
            context
                .joint_constraints
                .push(crate::joint::JointConstraint {
                    id: crate::world::JointId {
                        world: world_id,
                        dense,
                    },
                    body_a,
                    body_b,
                    joint,
                });
        }
        Ok(())
    }

    fn stage_collision_events(&mut self) -> Result<(), CollisionError> {
        #[cfg(feature = "bench-internals")]
        let started = Instant::now();
        let result = self.collision_system.stage_lifecycle_events();
        #[cfg(feature = "bench-internals")]
        self.collision_system.record_events(started.elapsed());
        result
    }

    #[cfg(feature = "bench-internals")]
    #[doc(hidden)]
    pub fn set_joint_mass_cache_for_bench(&mut self, enabled: bool) {
        self.collision_system
            .set_joint_mass_cache_for_bench(enabled);
    }

    #[cfg(feature = "bench-internals")]
    #[doc(hidden)]
    pub fn set_joint_warm_start_for_bench(&mut self, enabled: bool) {
        self.collision_system
            .set_joint_warm_start_for_bench(enabled);
    }

    #[cfg(feature = "bench-internals")]
    #[doc(hidden)]
    pub fn set_partial_coloring_for_bench(&mut self, enabled: bool) {
        self.collision_system
            .set_partial_coloring_for_bench(enabled);
    }

    #[cfg(feature = "bench-internals")]
    #[doc(hidden)]
    pub fn set_reduced_fixed_tree_for_bench(&mut self, enabled: bool) {
        self.collision_system
            .set_reduced_fixed_tree_for_bench(enabled);
    }

    #[cfg(feature = "bench-internals")]
    #[doc(hidden)]
    pub fn set_joint_projection_for_bench(&mut self, enabled: bool) {
        self.collision_system
            .set_joint_projection_for_bench(enabled);
    }

    #[cfg(feature = "bench-internals")]
    #[doc(hidden)]
    pub fn set_joint_light_diagnostics_for_bench(&mut self, enabled: bool) {
        self.collision_system
            .set_joint_light_diagnostics_for_bench(enabled);
    }

    #[cfg(feature = "bench-internals")]
    #[doc(hidden)]
    pub fn set_joint_adaptive_iterations_for_bench(&mut self, enabled: bool) {
        self.collision_system
            .set_joint_adaptive_iterations_for_bench(enabled);
    }

    #[cfg(feature = "bench-internals")]
    #[doc(hidden)]
    pub fn set_joint_alternating_velocity_sweeps_for_bench(&mut self, enabled: bool) {
        self.collision_system
            .set_joint_alternating_velocity_sweeps_for_bench(enabled);
    }

    #[cfg(feature = "bench-internals")]
    #[doc(hidden)]
    pub fn set_joint_adaptive_velocity_max_for_bench(&mut self, max: u32) {
        self.collision_system
            .set_joint_adaptive_velocity_max_for_bench(max);
    }

    #[cfg(feature = "bench-internals")]
    #[doc(hidden)]
    pub fn set_joint_global_velocity_pcg_for_bench(
        &mut self,
        max_iterations: u32,
        incomplete_cholesky_preconditioner: bool,
    ) {
        self.collision_system
            .set_joint_global_velocity_pcg_for_bench(
                max_iterations,
                incomplete_cholesky_preconditioner,
            );
    }

    #[cfg(feature = "bench-internals")]
    #[doc(hidden)]
    pub fn set_joint_global_velocity_block_pcg_for_bench(&mut self, max_iterations: u32) {
        self.collision_system
            .set_joint_global_velocity_block_pcg_for_bench(max_iterations);
    }

    #[cfg(feature = "bench-internals")]
    #[doc(hidden)]
    pub fn set_joint_velocity_relaxation_for_bench(&mut self, relaxation: f64) {
        self.collision_system
            .set_joint_velocity_relaxation_for_bench(relaxation);
    }

    #[cfg(feature = "bench-internals")]
    #[doc(hidden)]
    pub fn set_joint_global_position_pcg_for_bench(
        &mut self,
        max_iterations: u32,
        incomplete_cholesky_preconditioner: bool,
    ) {
        self.collision_system
            .set_joint_global_position_pcg_for_bench(
                max_iterations,
                incomplete_cholesky_preconditioner,
            );
    }

    #[cfg(feature = "bench-internals")]
    #[doc(hidden)]
    pub fn collision_step_metrics(&self) -> crate::collision::CollisionStepMetrics {
        let mut metrics = self.collision_system.benchmark_metrics();
        let query_index_bytes = self
            .query_index
            .as_ref()
            .map_or(0, SpatialIndex::retained_capacity_bytes)
            .saturating_add(
                self.query_proxies.capacity() * std::mem::size_of::<(ColliderId, Aabb)>(),
            )
            .saturating_add(
                self.unbounded_query_colliders.capacity() * std::mem::size_of::<ColliderId>(),
            );
        metrics.committed_query_index_bytes = query_index_bytes;
        metrics
    }

    #[cfg(feature = "bench-internals")]
    #[doc(hidden)]
    pub fn physics_step_metrics(&self) -> PhysicsStepMetrics {
        if !self.collision_system.step_metrics_committed() {
            return PhysicsStepMetrics::default();
        }
        PhysicsStepMetrics {
            collision: self.collision_step_metrics(),
            joints: self.collision_system.joint_solver_metrics(),
        }
    }

    #[cfg(feature = "bench-internals")]
    fn energy_components(&self) -> crate::joint::diagnostics::EnergyComponents {
        let mut energy = crate::joint::diagnostics::EnergyComponents::default();
        for (_, entry) in self.bodies.iter() {
            energy.add_assign(crate::collision::solver::body_energy_components(
                &entry.body,
                *entry.body.state(),
                self.central_gravity,
            ));
        }
        energy
    }

    fn required_body_index_slots(context: &StepContext) -> Result<usize, CollisionError> {
        context
            .predicted_states
            .iter()
            .try_fold(0_usize, |required, (body_id, _)| {
                let slot =
                    usize::try_from(body_id.index()).map_err(|_| CollisionError::GeometryBounds)?;
                let end = slot.checked_add(1).ok_or(CollisionError::GeometryBounds)?;
                Ok(required.max(end))
            })
    }

    fn rebuild_body_indices(context: &mut StepContext) -> Result<(), CollisionError> {
        if context.body_indices_ready {
            return Ok(());
        }
        let required_slots = Self::required_body_index_slots(context)?;
        if context.body_indices.len() < required_slots {
            context
                .body_indices
                .try_reserve(required_slots - context.body_indices.len())
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            context.body_indices.resize(required_slots, None);
        }
        context
            .body_indices
            .iter_mut()
            .for_each(|slot| *slot = None);
        for (index, (body_id, _)) in context.predicted_states.iter().copied().enumerate() {
            let slot =
                usize::try_from(body_id.index()).map_err(|_| CollisionError::GeometryBounds)?;
            *context
                .body_indices
                .get_mut(slot)
                .ok_or(CollisionError::InvalidContact)? = Some((body_id, index));
        }
        context.body_indices_ready = true;
        Ok(())
    }

    fn prepare_solver_body_indices(context: &mut StepContext) -> Result<(), CollisionError> {
        let required_slots = Self::required_body_index_slots(context)?;
        if context.solver_body_indices.len() < required_slots {
            context
                .solver_body_indices
                .try_reserve(required_slots - context.solver_body_indices.len())
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            context.solver_body_indices.resize(required_slots, None);
        }
        context
            .solver_body_indices
            .iter_mut()
            .for_each(|slot| *slot = None);
        context.body_types.clear();
        context.solver_bodies.clear();
        context.constrained_starts.clear();
        context.solver_body_predictions.clear();
        Ok(())
    }

    fn process_collision_candidates(
        &mut self,
        worker_count: usize,
        executor: &impl StepExecutor,
        dt: f64,
    ) -> Result<(), CollisionError> {
        let candidate_count = self.collision_system.candidate_count();
        let has_active_joints = self
            .joints
            .iter()
            .any(|(_, entry)| entry.joint.common().enabled);
        if candidate_count == 0 && !has_active_joints {
            self.collision_system.solve_contacts(
                worker_count,
                executor,
                dt,
                self.central_gravity,
            )?;
            self.stage_collision_events()?;
            return Ok(());
        }
        let context = &mut self.collision_system.context;
        Self::rebuild_body_indices(context)?;
        Self::prepare_solver_body_indices(context)?;
        if has_active_joints {
            #[cfg(feature = "bench-internals")]
            let joint_prepare_started = Instant::now();
            self.prepare_joint_constraints()?;
            #[cfg(feature = "bench-internals")]
            self.collision_system
                .record_joint_constraint_prepare(joint_prepare_started.elapsed());
        }

        if candidate_count == 0 {
            self.collision_system.solve_contacts(
                worker_count,
                executor,
                dt,
                self.central_gravity,
            )?;
            self.stage_collision_events()?;
            return Ok(());
        }

        self.collision_system.context.manifolds.clear();
        self.collision_system
            .context
            .manifolds
            .try_reserve(candidate_count)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        self.collision_system
            .context
            .interactions
            .try_reserve(candidate_count)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        for candidate_index in 0..candidate_count {
            let pair = self
                .collision_system
                .candidate(candidate_index)
                .ok_or(CollisionError::InvalidContact)?;
            let collider_a = self
                .colliders
                .get(pair.a.dense)
                .filter(|_| pair.a.world == self.world_id)
                .ok_or(CollisionError::InvalidContact)?;
            let collider_b = self
                .colliders
                .get(pair.b.dense)
                .filter(|_| pair.b.world == self.world_id)
                .ok_or(CollisionError::InvalidContact)?;
            if self
                .collision_system
                .context
                .suppresses_joint_collision(collider_a.body, collider_b.body)
            {
                continue;
            }
            let body_a_index = Self::predicted_body_index(
                &self.collision_system.context.body_indices,
                self.world_id,
                collider_a.body,
            )?;
            let body_b_index = Self::predicted_body_index(
                &self.collision_system.context.body_indices,
                self.world_id,
                collider_b.body,
            )?;
            let body_a = self
                .collision_system
                .context
                .predicted_states
                .get(body_a_index)
                .ok_or(CollisionError::InvalidContact)?
                .1
                .transform();
            let body_b = self
                .collision_system
                .context
                .predicted_states
                .get(body_b_index)
                .ok_or(CollisionError::InvalidContact)?
                .1
                .transform();
            let transform_a = body_a
                .compose(&collider_a.collider.local_transform())
                .map_err(|_| CollisionError::ArithmeticOverflow)?;
            let transform_b = body_b
                .compose(&collider_b.collider.local_transform())
                .map_err(|_| CollisionError::ArithmeticOverflow)?;
            #[cfg(feature = "bench-internals")]
            let started = Instant::now();
            let manifold = self.collision_system.detect(
                pair,
                collider_a.collider.shape(),
                &transform_a,
                &body_a,
                collider_a.collider.mode(),
                collider_a.collider.query_bvh(),
                collider_a.collider.query_vertices(),
                collider_b.collider.shape(),
                &transform_b,
                &body_b,
                collider_b.collider.mode(),
                collider_b.collider.query_bvh(),
                collider_b.collider.query_vertices(),
            )?;
            #[cfg(feature = "bench-internals")]
            self.collision_system.record_narrow_phase(
                started.elapsed(),
                manifold.as_ref().map_or(0, ContactManifold::len),
            );
            if let Some(manifold) = manifold {
                let sensor = collider_a.collider.mode() == crate::collider::CollisionMode::Sensor
                    || collider_b.collider.mode() == crate::collider::CollisionMode::Sensor;
                if sensor {
                    upsert_interaction(
                        &mut self.collision_system.context.interactions,
                        &mut self.collision_system.context.interaction_indices,
                        CollisionInteraction {
                            pair,
                            sensor: true,
                            transient: false,
                            contact: None,
                            impact: false,
                        },
                    )?;
                } else {
                    self.collision_system.context.manifolds.push(manifold);
                }
            }
        }

        self.collision_system.context.solver_manifolds.clear();
        self.collision_system
            .context
            .solver_manifolds
            .try_reserve(self.collision_system.context.manifolds.len())
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        for manifold_index in 0..self.collision_system.context.manifolds.len() {
            let manifold = self.collision_system.context.manifolds[manifold_index];
            let (body_id_a, body_id_b, mode_a, mode_b, material_a, material_b) = {
                let collider_a = self
                    .colliders
                    .get(manifold.pair.a.dense)
                    .filter(|_| manifold.pair.a.world == self.world_id)
                    .ok_or(CollisionError::InvalidContact)?;
                let collider_b = self
                    .colliders
                    .get(manifold.pair.b.dense)
                    .filter(|_| manifold.pair.b.world == self.world_id)
                    .ok_or(CollisionError::InvalidContact)?;
                (
                    collider_a.body,
                    collider_b.body,
                    collider_a.collider.mode(),
                    collider_b.collider.mode(),
                    collider_a.collider.material(),
                    collider_b.collider.material(),
                )
            };
            let prediction_a = Self::predicted_body_index(
                &self.collision_system.context.body_indices,
                self.world_id,
                body_id_a,
            )?;
            let prediction_b = Self::predicted_body_index(
                &self.collision_system.context.body_indices,
                self.world_id,
                body_id_b,
            )?;
            let type_a = self
                .bodies
                .get(body_id_a.dense)
                .filter(|_| body_id_a.world == self.world_id)
                .map(|entry| entry.body.physics().body_type())
                .ok_or(CollisionError::InvalidContact)?;
            let type_b = self
                .bodies
                .get(body_id_b.dense)
                .filter(|_| body_id_b.world == self.world_id)
                .map(|entry| entry.body.physics().body_type())
                .ok_or(CollisionError::InvalidContact)?;
            if type_a != BodyType::Dynamic && type_b != BodyType::Dynamic {
                continue;
            }
            if mode_a == crate::collider::CollisionMode::Sensor
                || mode_b == crate::collider::CollisionMode::Sensor
            {
                return Err(CollisionError::InvalidContact);
            }
            let material = self.combined_solver_material(material_a, material_b)?;
            let body_a = self.solver_body_index(body_id_a, prediction_a)?;
            let body_b = self.solver_body_index(body_id_b, prediction_b)?;
            self.collision_system
                .context
                .solver_manifolds
                .push(SolverManifold {
                    body_a,
                    body_b,
                    manifold,
                    material,
                });
        }
        self.collision_system
            .solve_contacts(worker_count, executor, dt, self.central_gravity)?;

        for manifold in self.collision_system.context.manifolds.iter().copied() {
            let collider_a = self
                .colliders
                .get(manifold.pair.a.dense)
                .filter(|_| manifold.pair.a.world == self.world_id)
                .ok_or(CollisionError::InvalidContact)?;
            let collider_b = self
                .colliders
                .get(manifold.pair.b.dense)
                .filter(|_| manifold.pair.b.world == self.world_id)
                .ok_or(CollisionError::InvalidContact)?;
            let body_a_index = Self::predicted_body_index(
                &self.collision_system.context.body_indices,
                self.world_id,
                collider_a.body,
            )?;
            let body_b_index = Self::predicted_body_index(
                &self.collision_system.context.body_indices,
                self.world_id,
                collider_b.body,
            )?;
            let body_a = self
                .collision_system
                .context
                .predicted_states
                .get(body_a_index)
                .ok_or(CollisionError::InvalidContact)?
                .1
                .transform();
            let body_b = self
                .collision_system
                .context
                .predicted_states
                .get(body_b_index)
                .ok_or(CollisionError::InvalidContact)?
                .1
                .transform();
            let mut deepest = None;
            let mut total_impulse = 0.0;
            let mut impact = false;
            for point in manifold.points().iter().copied() {
                total_impulse += point.normal_impulse;
                if !total_impulse.is_finite() {
                    return Err(CollisionError::ArithmeticOverflow);
                }
                impact |= point.normal_impulse_increase_sum > 0.0;
                if deepest.is_none_or(|current: crate::collision::ContactPoint| {
                    point.penetration > current.penetration
                }) {
                    deepest = Some(point);
                }
            }
            // Solver updates can add and then completely cancel an impulse.
            // Such a manifold has no resultant impact to publish.
            impact &= total_impulse > 0.0;
            let deepest = deepest.ok_or(CollisionError::InvalidContact)?;
            let point_a = body_a.transform_point(deepest.local_a);
            let point_b = body_b.transform_point(deepest.local_b);
            let point = point_a * 0.5 + point_b * 0.5;
            if !point.is_finite() {
                return Err(CollisionError::ArithmeticOverflow);
            }
            upsert_interaction(
                &mut self.collision_system.context.interactions,
                &mut self.collision_system.context.interaction_indices,
                CollisionInteraction {
                    pair: manifold.pair,
                    sensor: false,
                    transient: false,
                    contact: Some(ContactInfo {
                        point,
                        normal: deepest.normal,
                        penetration: deepest.penetration,
                        normal_impulse: total_impulse,
                    }),
                    impact,
                },
            )?;
        }
        self.stage_collision_events()?;
        Ok(())
    }

    fn predicted_body_index(
        body_indices: &[Option<(BodyId, usize)>],
        world_id: u64,
        id: BodyId,
    ) -> Result<usize, CollisionError> {
        if id.world != world_id {
            return Err(CollisionError::InvalidContact);
        }
        let slot = usize::try_from(id.index()).map_err(|_| CollisionError::InvalidContact)?;
        body_indices
            .get(slot)
            .and_then(|entry| *entry)
            .filter(|(current, _)| *current == id)
            .map(|(_, index)| index)
            .ok_or(CollisionError::InvalidContact)
    }

    fn solver_body_index(
        &mut self,
        body_id: BodyId,
        prediction_index: usize,
    ) -> Result<usize, CollisionError> {
        Self::solver_body_index_in_context(
            &self.bodies,
            self.world_id,
            &mut self.collision_system.context,
            body_id,
            prediction_index,
        )
    }

    fn solver_body_index_in_context(
        bodies: &DensePool<BodyEntry>,
        world_id: u64,
        context: &mut StepContext,
        body_id: BodyId,
        prediction_index: usize,
    ) -> Result<usize, CollisionError> {
        let slot = usize::try_from(body_id.index()).map_err(|_| CollisionError::InvalidContact)?;
        if let Some((current, index)) = context.solver_body_indices.get(slot).copied().flatten()
            && current == body_id
        {
            return Ok(index);
        }
        let entry = bodies
            .get(body_id.dense)
            .filter(|_| body_id.world == world_id)
            .ok_or(CollisionError::InvalidContact)?;
        let body = entry.body.motion();
        let constrained_start = (entry.first_collider.is_none()
            && entry.cluster.is_none()
            && body.ccd_mode() == CcdMode::Discrete)
            .then_some(SolverBody { body });
        let state = context
            .predicted_states
            .get(prediction_index)
            .filter(|(current, _)| *current == body_id)
            .map(|(_, state)| *state)
            .ok_or(CollisionError::InvalidContact)?;
        let index = context.solver_bodies.len();
        context
            .solver_bodies
            .try_reserve(1)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        context
            .solver_body_predictions
            .try_reserve(1)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        context
            .body_types
            .try_reserve(1)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        *context
            .solver_body_indices
            .get_mut(slot)
            .ok_or(CollisionError::InvalidContact)? = Some((body_id, index));
        context
            .solver_bodies
            .push(SolverBody::from_prediction(body, state));
        context
            .constrained_starts
            .try_reserve(1)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        context.constrained_starts.push(constrained_start);
        context.solver_body_predictions.push(prediction_index);
        context.body_types.push(body.physics().body_type());
        Ok(index)
    }

    fn combined_solver_material(
        &self,
        material_a: Option<MaterialId>,
        material_b: Option<MaterialId>,
    ) -> Result<SolverMaterial, CollisionError> {
        let fallback = PhysicsMaterial::default();
        let material_a = material_a
            .and_then(|id| self.materials.get(id))
            .copied()
            .unwrap_or(fallback);
        let material_b = material_b
            .and_then(|id| self.materials.get(id))
            .copied()
            .unwrap_or(fallback);
        let dynamic_friction = material_a.friction().sqrt() * material_b.friction().sqrt();
        let static_friction =
            material_a.static_friction().sqrt() * material_b.static_friction().sqrt();
        if !dynamic_friction.is_finite() || !static_friction.is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
        Ok(SolverMaterial {
            restitution: material_a.restitution().max(material_b.restitution()),
            static_friction,
            dynamic_friction,
        })
    }

    fn validate_collision_inputs(&self) -> Result<(), CollisionError> {
        self.collision_system.config().validate()?;
        for (_, entry) in self.colliders.iter() {
            if entry
                .collider
                .material()
                .is_some_and(|id| !self.materials.contains(id))
            {
                return Err(CollisionError::InvalidMaterial);
            }
            let body = self
                .bodies
                .get(entry.body.dense)
                .filter(|_| entry.body.world == self.world_id)
                .ok_or(CollisionError::InvalidContact)?;
            if body.body.physics().body_type() == crate::rigid_body::BodyType::Dynamic
                && !entry.collider.shape().is_convex()
            {
                return Err(CollisionError::UnsupportedDynamicGeometry);
            }
        }
        Ok(())
    }

    /// Fills balanced, ordered body snapshots into reusable shard buffers.
    ///
    /// This low-level helper does not prepare persistent force sources. Use
    /// [`Self::step_with_shards`] to execute a complete step.
    #[doc(hidden)]
    pub fn fill_step_shards(&self, shards: &mut [Vec<BodyStep>]) -> Result<(), StepError> {
        self.fill_step_shards_with_executor(shards, &SequentialExecutor)
    }

    fn fill_step_shards_with_executor<E: StepExecutor>(
        &self,
        shards: &mut [Vec<BodyStep>],
        executor: &E,
    ) -> Result<(), StepError> {
        self.fill_step_shards_with_executor_at_dt(shards, executor, None, true)
    }

    fn fill_step_shards_with_executor_at_dt<E: StepExecutor>(
        &self,
        shards: &mut [Vec<BodyStep>],
        executor: &E,
        dt: Option<f64>,
        apply_aerodynamics: bool,
    ) -> Result<(), StepError> {
        if shards.is_empty() {
            return Err(StepError::InvalidStepResults);
        }
        for shard in shards.iter_mut() {
            shard.clear();
        }

        let base_size = self.body_count() / shards.len();
        let remainder = self.body_count() % shards.len();
        for (index, shard) in shards.iter_mut().enumerate() {
            let required = base_size + usize::from(index < remainder);
            shard
                .try_reserve(required)
                .map_err(|_| StepError::Collision(CollisionError::SpatialIndexCapacityExceeded))?;
        }
        let mut shard_index = 0;
        let mut shard_remaining = base_size + usize::from(remainder > 0);

        for (dense_id, entry) in self.bodies.iter() {
            while shard_remaining == 0 {
                shard_index += 1;
                shard_remaining = base_size + usize::from(shard_index < remainder);
            }
            let body_id = BodyId {
                world: self.world_id,
                dense: dense_id,
            };
            shards[shard_index].push(BodyStep {
                body_id,
                source: entry.body.motion(),
                body_revision: self.body_revision,
                predicted: None,
                dt: None,
                error: None,
                prepared: entry.body.motion(),
            });
            shard_remaining -= 1;
        }
        if !matches!(self.atmosphere, Atmosphere::Vacuum)
            || self.central_gravity.is_some()
            || !self.gravity_fields.is_empty()
        {
            let atmosphere = self.atmosphere;
            executor
                .for_each_chunk_mut(shards, 1, |_, chunks| {
                    for step in chunks.iter_mut().flatten() {
                        if apply_aerodynamics
                            && !matches!(atmosphere, Atmosphere::Vacuum)
                            && let Some(entry) = self.bodies.get(step.body_id.dense)
                            && (!entry.body.aerodynamic_surfaces.is_empty()
                                || !entry.body.base_pressure_patches.is_empty()
                                || assembly_aerodynamics::wings_configured(entry, &self.parts)
                                || assembly_aerodynamics::hull_configured(entry, &self.parts)
                                || !matches!(
                                    entry.body.aerodynamics(),
                                    crate::aerodynamics::Aerodynamics::None
                                )
                                || assembly_aerodynamics::configured(
                                    entry,
                                    self.part_aerodynamic_geometry.as_ref(),
                                ))
                        {
                            let staged = self
                                .aerodynamics_body_updates
                                .iter()
                                .find(|update| update.id == step.body_id);
                            if staged.is_none() && entry.body.potential_sphere.is_some() {
                                step.error = Some(StepError::Aerodynamics {
                                    body_id: step.body_id,
                                    cause: crate::aerodynamics::AerodynamicsError::UnsupportedModel,
                                });
                                continue;
                            }
                            let mut temporary_cache =
                                crate::aerodynamics::cache::PreparedGeometryCache::default();
                            let local_wave = if staged.is_none() {
                                if let crate::aerodynamics::Aerodynamics::VoxelWave(model) =
                                    entry.body.aerodynamics()
                                {
                                    Self::prepare_voxel_wave(
                                        model,
                                        step.body_id,
                                        &entry.body.motion(),
                                        atmosphere,
                                        &mut temporary_cache,
                                    )
                                } else {
                                    Ok(None)
                                }
                            } else {
                                Ok(None)
                            };
                            let mut temporary_assembly = None;
                            let local_assembly = if staged.is_none()
                                && (assembly_aerodynamics::wave_configured(
                                    entry,
                                    self.part_aerodynamic_geometry.as_ref(),
                                ) || assembly_aerodynamics::shielding_configured(
                                    entry,
                                    &self.parts,
                                    self.part_aerodynamic_geometry.as_ref(),
                                )) {
                                assembly_aerodynamics::prepare(
                                    step.body_id,
                                    entry,
                                    &self.parts,
                                    self.part_aerodynamic_geometry.as_ref(),
                                    &self.colliders,
                                    &mut temporary_assembly,
                                    atmosphere,
                                )
                            } else {
                                Ok(None)
                            };
                            step.error = local_wave
                                .and_then(|local_wave| {
                                    local_assembly.and_then(|local_assembly| {
                                        let inputs = AerodynamicsInputs::new(self.mesh_sound_speed)
                                            .with_base_pressure(&entry.body.base_pressure_patches)
                                            .with_cluster(entry.cluster.as_ref())
                                            .with_timestep(dt);
                                        let inputs = if let Some(update) = staged {
                                            inputs.with_loads(&update.loads)
                                        } else {
                                            inputs
                                                .with_assembly(local_assembly.as_ref())
                                                .with_voxel_wave(local_wave.as_ref())
                                        };
                                        Self::apply_aerodynamics(
                                            atmosphere,
                                            entry.body.aerodynamics(),
                                            &entry.body.aerodynamic_surfaces,
                                            step.body_id,
                                            &mut step.prepared,
                                            &self.parts,
                                            inputs,
                                        )
                                    })
                                })
                                .err();
                        }
                        if step.error.is_none() {
                            step.error = Self::apply_gravity(
                                self.central_gravity,
                                &self.gravity_fields,
                                step.body_id,
                                &mut step.prepared,
                            )
                            .err();
                        }
                    }
                })
                .map_err(|_| StepError::Collision(CollisionError::WorkerFailed))?;
            if let Some(error) = shards.iter().flatten().find_map(|step| step.error) {
                return Err(error);
            }
        }
        Ok(())
    }

    /// Validates all shard results before committing any body state.
    #[doc(hidden)]
    pub fn commit_step_shards(&mut self, shards: &[Vec<BodyStep>]) -> Result<(), StepError> {
        self.commit_step_shards_with_workers(shards, shards.len().max(1))
    }

    /// Validates shard results using the built-in sequential executor.
    ///
    /// Use [`Self::commit_step_shards_with_executor`] to select another executor.
    #[doc(hidden)]
    pub fn commit_step_shards_with_workers(
        &mut self,
        shards: &[Vec<BodyStep>],
        _worker_count: usize,
    ) -> Result<(), StepError> {
        self.commit_step_shards_with_executor(shards, &SequentialExecutor, 1)
    }

    /// Validates shard results and delegates independent collision work to an executor.
    ///
    /// This low-level commit helper does not prepare persistent force sources. Use
    /// [`Self::step_with_shards`] to execute a complete step.
    #[doc(hidden)]
    pub fn commit_step_shards_with_executor<E: StepExecutor>(
        &mut self,
        shards: &[Vec<BodyStep>],
        executor: &E,
        worker_count: usize,
    ) -> Result<(), StepError> {
        self.commit_step_shards_inner(shards, executor, worker_count, false)
    }

    fn commit_step_shards_inner<E: StepExecutor>(
        &mut self,
        shards: &[Vec<BodyStep>],
        executor: &E,
        worker_count: usize,
        full_step: bool,
    ) -> Result<(), StepError> {
        let mut steps = shards.iter().flatten();
        let mut step_dt = None;
        for (dense_id, entry) in self.bodies.iter() {
            let Some(step) = steps.next() else {
                return Err(StepError::InvalidStepResults);
            };
            if let Some(error) = step.error {
                return Err(error);
            }
            if step.body_id
                != (BodyId {
                    world: self.world_id,
                    dense: dense_id,
                })
                || step.source != entry.body.motion()
                || step.body_revision != self.body_revision
                || step.predicted.is_none()
                || step.dt.is_none_or(|dt| !dt.is_finite() || dt <= 0.0)
            {
                return Err(StepError::InvalidStepResults);
            }
            if step_dt.is_some_and(|dt| Some(dt) != step.dt) {
                return Err(StepError::InvalidStepResults);
            }
            step_dt = step.dt;
        }
        let dt = step_dt.unwrap_or(0.0);
        if steps.next().is_some() {
            return Err(StepError::InvalidStepResults);
        }

        self.validate_collision_inputs()
            .map_err(StepError::Collision)?;
        self.prepare_gravity_updates(executor, worker_count)?;
        self.collision_system.begin_step();
        if let Err(error) = self.prepare_joint_collision_pairs() {
            self.collision_system.abort_step();
            self.rollback_gravity_updates();
            self.rollback_aerodynamics_updates();
            return Err(StepError::Collision(error));
        }
        for step in shards.iter().flatten() {
            if let Some(state) = step.predicted {
                self.collision_system.stage_prediction(step.body_id, state);
            }
        }
        let worker_count = worker_count.max(1);
        if let Err(error) = self.prepare_collision_candidates(dt, worker_count, executor) {
            self.collision_system.abort_step();
            self.rollback_gravity_updates();
            self.rollback_aerodynamics_updates();
            return Err(StepError::Collision(error));
        }
        if let Err(error) = self.process_collision_candidates(worker_count, executor, dt) {
            self.collision_system.abort_step();
            self.rollback_gravity_updates();
            self.rollback_aerodynamics_updates();
            return Err(StepError::Collision(error));
        }
        if let Err(error) = self.prepare_joint_break_commit() {
            self.collision_system.abort_step();
            self.rollback_gravity_updates();
            self.rollback_aerodynamics_updates();
            return Err(StepError::Collision(error));
        }
        if let Err(error) = self.stage_contact_impulses() {
            self.collision_system.abort_step();
            self.rollback_gravity_updates();
            self.rollback_aerodynamics_updates();
            return Err(StepError::Collision(error));
        }
        let splits = if full_step {
            match self.stage_automatic_weld_splits() {
                Ok(splits) => splits,
                Err(error) => {
                    self.collision_system.abort_step();
                    self.rollback_gravity_updates();
                    self.rollback_aerodynamics_updates();
                    return Err(error);
                }
            }
        } else {
            None
        };

        #[cfg(feature = "bench-internals")]
        let commit_started = Instant::now();
        self.commit_predicted_states();
        if let Some(splits) = splits {
            self.finish_weld_splits(splits);
        }
        self.commit_joint_breaks();
        self.collision_system.commit_step();
        self.clear_gravity_updates();
        #[cfg(feature = "bench-internals")]
        self.collision_system
            .record_commit(commit_started.elapsed());
        Ok(())
    }

    pub fn add_collider(
        &mut self,
        body: BodyId,
        collider: Collider,
    ) -> Result<ColliderId, WorldError> {
        if self
            .bodies
            .get(body.dense)
            .is_some_and(|entry| entry.cluster.is_some())
        {
            return Err(WorldError::InvalidBodyId);
        }
        self.add_collider_internal(body, collider)
    }

    fn add_collider_internal(
        &mut self,
        body: BodyId,
        mut collider: Collider,
    ) -> Result<ColliderId, WorldError> {
        if body.world != self.world_id {
            return Err(WorldError::InvalidBodyId);
        }
        if collider
            .material()
            .is_some_and(|material| !self.materials.contains(material))
        {
            return Err(WorldError::InvalidMaterialId);
        }
        let first = self
            .bodies
            .get(body.dense)
            .ok_or(WorldError::InvalidBodyId)?
            .first_collider;
        if self.bodies.get(body.dense).is_some_and(|entry| {
            entry.body.physics().body_type() == crate::rigid_body::BodyType::Dynamic
                && !collider.shape().is_convex()
        }) {
            return Err(WorldError::Collision(
                CollisionError::UnsupportedDynamicGeometry,
            ));
        }
        if first.is_some_and(|id| self.colliders.get(id.dense).is_none()) {
            return Err(WorldError::InvalidColliderId);
        }
        collider.prepare_query_geometry()?;
        let has_finite_bounds = collider.local_bounds().ok().flatten().is_some();
        if !has_finite_bounds {
            self.unbounded_query_colliders
                .try_reserve(1)
                .map_err(|_| WorldError::Collision(CollisionError::SpatialIndexCapacityExceeded))?;
        }

        let dense = self
            .colliders
            .insert(ColliderEntry {
                collider,
                body,
                prev: None,
                next: first,
                chunk: None,
            })
            .map_err(WorldError::from)?;
        let id = ColliderId {
            world: self.world_id,
            dense,
        };

        if let Some(first) = first
            && let Some(entry) = self.colliders.get_mut(first.dense)
        {
            entry.prev = Some(id);
        }
        if let Some(entry) = self.bodies.get_mut(body.dense) {
            entry.first_collider = Some(id);
            entry.collider_aerodynamic_mesh = None;
            entry.mesh_aerodynamics = None;
            entry.body.invalidate_collider_base_patches();
        }
        if let Some(caches) = &mut self.assembly_aerodynamics {
            caches.remove(&body);
        }
        if has_finite_bounds {
            self.update_query_index_for_collider(id);
        } else {
            self.unbounded_query_colliders.push(id);
        }
        self.collision_system.mark_spatial_dirty();
        Ok(id)
    }

    /// Updates the deformed local vertices for a skinned-mesh collider.
    ///
    /// The pose is fixed for each physics step. Changes between steps are
    /// discrete, and CCD does not interpolate unknown bone trajectories.
    pub fn set_skin_pose(
        &mut self,
        id: ColliderId,
        matrices: &[Affine3],
    ) -> Result<(), CollisionError> {
        if id.world != self.world_id {
            return Err(CollisionError::InvalidSkinPose);
        }
        let entry = self
            .colliders
            .get_mut(id.dense)
            .ok_or(CollisionError::InvalidSkinPose)?;
        entry.collider.set_skin_pose(matrices)?;
        self.mark_query_index_dirty();
        self.collision_system.mark_spatial_dirty();
        self.collision_system.invalidate_collider_contacts(id);
        Ok(())
    }

    pub fn collider(&self, id: ColliderId) -> Option<&Collider> {
        if id.world != self.world_id {
            return None;
        }
        self.colliders.get(id.dense).map(|entry| &entry.collider)
    }

    pub fn collider_mut(&mut self, id: ColliderId) -> Option<&mut Collider> {
        if id.world != self.world_id {
            return None;
        }
        let collider = self.colliders.get(id.dense)?;
        if collider.chunk.is_some() {
            return None;
        }
        let body = collider.body;
        if self
            .bodies
            .get(body.dense)
            .is_some_and(|entry| entry.cluster.is_some())
        {
            return None;
        }
        if self.colliders.get(id.dense).is_some() {
            self.collision_system.invalidate_collider_contacts(id);
        }
        self.mark_query_index_dirty();
        self.collision_system.mark_spatial_dirty();
        if let Some(entry) = self.bodies.get_mut(body.dense) {
            entry.collider_aerodynamic_mesh = None;
            entry.mesh_aerodynamics = None;
            entry.body.invalidate_collider_base_patches();
        }
        if let Some(caches) = &mut self.assembly_aerodynamics {
            caches.remove(&body);
        }
        self.colliders
            .get_mut(id.dense)
            .map(|entry| &mut entry.collider)
    }

    pub fn collider_body(&self, id: ColliderId) -> Option<BodyId> {
        if id.world != self.world_id {
            return None;
        }
        self.colliders.get(id.dense).map(|entry| entry.body)
    }

    pub fn remove_collider(&mut self, id: ColliderId) -> Option<Collider> {
        if id.world != self.world_id {
            return None;
        }
        let collider = self.colliders.get(id.dense)?;
        if collider.chunk.is_some() {
            return None;
        }
        let body = collider.body;
        if self
            .bodies
            .get(body.dense)
            .is_some_and(|entry| entry.cluster.is_some())
        {
            return None;
        }
        self.remove_collider_internal(id)
    }

    fn remove_collider_internal(&mut self, id: ColliderId) -> Option<Collider> {
        if id.world != self.world_id {
            return None;
        }
        let (body, prev, next) = {
            let entry = self.colliders.get(id.dense)?;
            (entry.body, entry.prev, entry.next)
        };
        let body_entry = self.bodies.get(body.dense)?;
        if prev.is_none() && body_entry.first_collider != Some(id) {
            return None;
        }
        if prev.is_some_and(|prev| {
            self.colliders
                .get(prev.dense)
                .is_none_or(|entry| entry.body != body || entry.next != Some(id))
        }) || next.is_some_and(|next| {
            self.colliders
                .get(next.dense)
                .is_none_or(|entry| entry.body != body || entry.prev != Some(id))
        }) {
            return None;
        }

        if !self.query_index_dirty {
            let remove_failed = self
                .query_index
                .as_mut()
                .is_some_and(|index| index.remove(id).is_err());
            if remove_failed {
                self.mark_query_index_dirty();
            }
        }
        self.unbounded_query_colliders
            .retain(|collider| *collider != id);

        let removed = self.colliders.remove(id.dense)?;
        if let Some(entry) = self.bodies.get_mut(body.dense) {
            entry.collider_aerodynamic_mesh = None;
            entry.mesh_aerodynamics = None;
            entry.body.invalidate_collider_base_patches();
        }
        if let Some(caches) = &mut self.assembly_aerodynamics {
            caches.remove(&body);
        }
        if let Some(prev) = prev {
            if let Some(entry) = self.colliders.get_mut(prev.dense) {
                entry.next = next;
            }
        } else if let Some(entry) = self.bodies.get_mut(body.dense) {
            entry.first_collider = next;
        }
        if let Some(next) = next
            && let Some(entry) = self.colliders.get_mut(next.dense)
        {
            entry.prev = prev;
        }
        self.collision_system.mark_spatial_dirty();
        self.collision_system.invalidate_collider_contacts(id);
        Some(removed.collider)
    }

    pub fn collider_count(&self) -> usize {
        self.colliders.len()
    }

    pub fn remove_body(&mut self, id: BodyId) -> Option<RigidBody> {
        if id.world != self.world_id {
            return None;
        }
        let cluster = self.bodies.get(id.dense)?.cluster.clone();
        let mut next = self.bodies.get(id.dense)?.first_collider;
        while let Some(collider) = next {
            next = self.colliders.get(collider.dense)?.next;
            self.remove_collider_internal(collider)?;
        }
        self.remove_body_chunks(id);
        loop {
            let joint = self.joints.iter().find_map(|(dense, entry)| {
                (entry.body_a == id || entry.body_b == id).then_some(JointId {
                    world: self.world_id,
                    dense,
                })
            });
            let Some(joint) = joint else {
                break;
            };
            self.joints.remove(joint.dense)?;
            self.collision_system.invalidate_joint_warm_start(joint);
        }
        loop {
            let source = self.force_sources.iter().find_map(|(dense, source)| {
                (source.body == id).then_some(ForceSourceId {
                    world: self.world_id,
                    dense,
                })
            });
            let Some(source) = source else {
                break;
            };
            self.force_sources.remove(source.dense)?;
        }
        let removed = self.bodies.remove(id.dense).map(|entry| entry.body)?;
        if let Some(workspaces) = &mut self.owner_pressure {
            workspaces.remove(&id);
        }
        if let Some(cluster) = cluster {
            for weld in cluster.welds {
                self.welds.remove(weld.dense);
            }
            for part in cluster.parts {
                self.parts.remove(part.dense);
                if let Some(meshes) = &mut self.part_aerodynamic_geometry {
                    meshes.remove(&part);
                }
            }
        }
        if let Some(caches) = &mut self.assembly_aerodynamics {
            caches.remove(&id);
        }
        Some(removed)
    }

    /// Iterates over bodies in dense storage order, which may change after removal.
    pub fn bodies(&self) -> impl Iterator<Item = (BodyId, &RigidBody)> {
        self.bodies.iter().map(|(dense, entry)| {
            (
                BodyId {
                    world: self.world_id,
                    dense,
                },
                &entry.body,
            )
        })
    }

    /// Mutably iterates over non-cluster bodies in dense storage order.
    /// The order may change after removal; cluster-managed bodies are omitted.
    pub fn bodies_mut(&mut self) -> impl Iterator<Item = (BodyId, &mut RigidBody)> {
        self.body_revision = self.body_revision.wrapping_add(1);
        self.mark_query_index_dirty();
        self.collision_system.mark_spatial_dirty();
        self.bodies.iter_mut().filter_map(|(dense, entry)| {
            entry.cluster.is_none().then_some((
                BodyId {
                    world: self.world_id,
                    dense,
                },
                &mut entry.body,
            ))
        })
    }

    /// Iterates over colliders in dense storage order, which may change after removal.
    pub fn colliders(&self) -> impl Iterator<Item = (ColliderId, BodyId, &Collider)> {
        self.colliders.iter().map(|(dense, entry)| {
            (
                ColliderId {
                    world: self.world_id,
                    dense,
                },
                entry.body,
                &entry.collider,
            )
        })
    }

    /// Iterates over the colliders attached to `body` in linked-list order.
    pub fn colliders_of(
        &self,
        body: BodyId,
    ) -> Option<impl Iterator<Item = (ColliderId, &Collider)> + '_> {
        collider_entries_of(self.world_id, &self.bodies, &self.colliders, body)
            .map(|entries| entries.map(|(id, _, collider)| (id, collider)))
    }
}

impl Default for PhysicsWorld {
    fn default() -> Self {
        Self::new()
    }
}

fn validate_force_source_values(
    local_point: Vec3,
    local_force: Vec3,
) -> Result<(), ForceSourceError> {
    if !local_point.is_finite() || !local_force.is_finite() {
        return Err(ForceSourceError::NonFiniteValue);
    }
    Ok(())
}

pub(crate) fn transform_joint_frame(
    target_transform: crate::transform::Transform,
    frame: JointFrame,
) -> Result<JointFrame, crate::transform::TransformError> {
    let frame_transform =
        crate::transform::Transform::new(frame.local_anchor, frame.local_rotation)?;
    let transformed = target_transform.compose(&frame_transform)?;
    Ok(JointFrame::new(
        transformed.translation(),
        transformed.rotation(),
    ))
}

fn body_point_velocity(
    mut body: BodyMotion,
    state: BodyState,
    point: Vec3,
) -> Result<Vec3, CollisionError> {
    body.set_state_preserving_forces(state);
    let center = body
        .world_center_of_mass()
        .map_err(|_| CollisionError::ArithmeticOverflow)?
        .unwrap_or(state.transform().translation());
    let angular_velocity = state
        .angular_velocity()
        .checked_cross(
            point
                .checked_sub(center)
                .ok_or(CollisionError::ArithmeticOverflow)?,
        )
        .ok_or(CollisionError::ArithmeticOverflow)?;
    state
        .linear_velocity()
        .checked_add(angular_velocity)
        .ok_or(CollisionError::ArithmeticOverflow)
}

fn collider_entries_of<'a>(
    world_id: u64,
    bodies: &DensePool<BodyEntry>,
    colliders: &'a DensePool<ColliderEntry>,
    body: BodyId,
) -> Option<impl Iterator<Item = (ColliderId, BodyId, &'a Collider)> + 'a> {
    if body.world != world_id {
        return None;
    }
    let mut next = bodies.get(body.dense)?.first_collider;
    Some(std::iter::from_fn(move || {
        let id = next?;
        let entry = colliders.get(id.dense)?;
        next = entry.next;
        Some((id, entry.body, &entry.collider))
    }))
}

fn update_query_index_for_body(
    index: &mut SpatialIndex,
    colliders: &DensePool<ColliderEntry>,
    world_id: u64,
    body_id: BodyId,
    body: &BodyEntry,
) -> Result<(), CollisionError> {
    let mut next = body.first_collider;
    while let Some(collider_id) = next {
        if collider_id.world != world_id {
            return Err(CollisionError::InvalidContact);
        }
        let entry = colliders
            .get(collider_id.dense)
            .filter(|entry| entry.body == body_id)
            .ok_or(CollisionError::InvalidContact)?;
        if let Some(bounds) = entry
            .collider
            .world_bounds(&body.body.state().transform())
            .map_err(|_| CollisionError::GeometryBounds)?
        {
            if !index.contains_bounds(collider_id, bounds)? {
                index.update(collider_id, expanded_query_bounds(bounds)?)?;
            }
        } else {
            index.remove(collider_id)?;
        }
        next = entry.next;
    }
    Ok(())
}

fn expanded_query_bounds(bounds: Aabb) -> Result<Aabb, CollisionError> {
    const MARGIN: f64 = 0.1;
    let margin = Vec3::splat(MARGIN);
    let min = bounds
        .min()
        .checked_sub(margin)
        .ok_or(CollisionError::GeometryBounds)?;
    let max = bounds
        .max()
        .checked_add(margin)
        .ok_or(CollisionError::GeometryBounds)?;
    Aabb::new(min, max).map_err(|_| CollisionError::GeometryBounds)
}

fn upsert_interaction(
    interactions: &mut Vec<CollisionInteraction>,
    indices: &mut Option<HashMap<CollisionPair, usize>>,
    interaction: CollisionInteraction,
) -> Result<(), CollisionError> {
    let indices = indices.get_or_insert_with(HashMap::new);
    if let Some(&index) = indices.get(&interaction.pair) {
        let existing = interactions
            .get_mut(index)
            .ok_or(CollisionError::InvalidContact)?;
        merge_interaction(existing, interaction)?;
        return Ok(());
    }
    interactions
        .try_reserve(1)
        .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
    indices
        .try_reserve(1)
        .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
    let index = interactions.len();
    interactions.push(interaction);
    indices.insert(interaction.pair, index);
    Ok(())
}

fn remove_transient_sensor_interaction(
    interactions: &mut Vec<CollisionInteraction>,
    indices: &mut Option<HashMap<CollisionPair, usize>>,
    pair: CollisionPair,
) -> Result<(), CollisionError> {
    let Some(index) = indices
        .as_ref()
        .and_then(|indices| indices.get(&pair).copied())
    else {
        return Ok(());
    };
    let interaction = interactions
        .get(index)
        .ok_or(CollisionError::InvalidContact)?;
    if !interaction.sensor || !interaction.transient {
        return Ok(());
    }
    interactions.remove(index);
    let indices = indices.as_mut().ok_or(CollisionError::InvalidContact)?;
    indices.clear();
    indices
        .try_reserve(interactions.len())
        .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
    for (index, interaction) in interactions.iter().enumerate() {
        if indices.insert(interaction.pair, index).is_some() {
            return Err(CollisionError::InvalidContact);
        }
    }
    Ok(())
}

fn remove_interactions_for_pairs(
    interactions: &mut Vec<CollisionInteraction>,
    indices: &mut Option<HashMap<CollisionPair, usize>>,
    pairs: &[CollisionPair],
) -> Result<(), CollisionError> {
    interactions.retain(|interaction| pairs.binary_search(&interaction.pair).is_err());
    let Some(indices) = indices else {
        return Ok(());
    };
    indices.clear();
    indices
        .try_reserve(interactions.len())
        .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
    for (index, interaction) in interactions.iter().enumerate() {
        if indices.insert(interaction.pair, index).is_some() {
            return Err(CollisionError::InvalidContact);
        }
    }
    Ok(())
}

fn merge_interaction(
    existing: &mut CollisionInteraction,
    incoming: CollisionInteraction,
) -> Result<(), CollisionError> {
    if existing.sensor != incoming.sensor {
        return Err(CollisionError::InvalidContact);
    }
    existing.transient &= incoming.transient;
    if let Some(contact) = incoming.contact
        && (!existing.impact || contact.normal_impulse > 0.0)
    {
        // A later resting manifold must not erase the contact that proves an
        // earlier CCD impact. Impact events require a positive actual impulse.
        existing.contact = Some(contact);
    }
    existing.impact |= incoming.impact;
    Ok(())
}

#[cfg(test)]
#[path = "world_tests.rs"]
mod tests;

#[cfg(test)]
fn test_atmosphere(state: crate::aerodynamics::AtmosphericState) -> Atmosphere {
    crate::aerodynamics::atmosphere::test_atmosphere(state)
}
