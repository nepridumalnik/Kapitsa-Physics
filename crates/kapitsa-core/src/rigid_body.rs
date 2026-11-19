pub mod error;
mod forces;
pub(crate) mod integrator;
mod physics;
pub(crate) mod solver_mass;
mod state;

pub use error::BodyError;
pub use forces::BodyForces;
pub use integrator::IntegrationError;
pub use physics::{BodyPhysics, BodyType};
pub use state::BodyState;

/// Continuous collision detection policy for a rigid body.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum CcdMode {
    Discrete,
    Continuous,
}

pub(crate) use integrator::{predict, predict_from_state, trajectory_bounds};

use kapitsa_math::{CholeskyError, Quat, Vec3};

use crate::{Error, mass::MassProperties, transform::Transform};

/// A rigid body's state, physical properties, and accumulated world-space forces.
///
/// The transform describes the body's world-space local origin. Its center of mass
/// may be offset from that origin by [`MassProperties::center_of_mass`].
#[derive(Debug)]
pub struct RigidBody {
    motion: BodyMotion,
    user_data: u64,
    aerodynamics: crate::aerodynamics::Aerodynamics,
    pub(crate) aerodynamic_geometry_identity: Option<std::sync::Arc<()>>,
    pub(crate) aerodynamic_surfaces: Vec<crate::aerodynamics::AerodynamicSurface>,
    pub(crate) base_pressure_patches: Vec<(
        crate::aerodynamics::pressure::PressurePatch,
        crate::aerodynamics::pressure::BasePressure,
    )>,
    pub(crate) base_pressure_triangles: Option<Vec<[Vec3; 3]>>,
    pub(crate) potential_sphere: Option<(Vec3, f64)>,
}

impl RigidBody {
    pub fn new_static(transform: Transform) -> Self {
        Self::from_motion(BodyMotion::new_static(transform))
    }

    pub fn new_kinematic(transform: Transform) -> Self {
        Self::from_motion(BodyMotion::new_kinematic(transform))
    }

    pub fn new_dynamic(transform: Transform, mass: MassProperties) -> Result<Self, Error> {
        BodyMotion::new_dynamic(transform, mass).map(Self::from_motion)
    }

    pub(crate) fn from_motion(motion: BodyMotion) -> Self {
        Self {
            motion,
            user_data: 0,
            aerodynamics: crate::aerodynamics::Aerodynamics::None,
            aerodynamic_geometry_identity: None,
            aerodynamic_surfaces: Vec::new(),
            base_pressure_patches: Vec::new(),
            base_pressure_triangles: None,
            potential_sphere: None,
        }
    }

    /// Returns the application-defined value stored with this body.
    pub fn user_data(&self) -> u64 {
        self.user_data
    }

    /// Stores an application-defined value on this body.
    ///
    /// The engine never interprets this value. It is not part of [`BodyMotion`]
    /// and is reset when a body is rebuilt from motion.
    pub fn set_user_data(&mut self, user_data: u64) {
        self.user_data = user_data;
    }

    pub fn aerodynamics(&self) -> &crate::aerodynamics::Aerodynamics {
        &self.aerodynamics
    }

    pub fn set_aerodynamics(&mut self, aerodynamics: crate::aerodynamics::Aerodynamics) {
        self.set_primary_aerodynamics(aerodynamics);
    }

    pub(crate) fn set_primary_aerodynamics(
        &mut self,
        aerodynamics: crate::aerodynamics::Aerodynamics,
    ) {
        self.base_pressure_patches.clear();
        self.base_pressure_triangles = None;
        self.potential_sphere = None;
        self.aerodynamic_geometry_identity = matches!(
            aerodynamics,
            crate::aerodynamics::Aerodynamics::Mesh(_)
                | crate::aerodynamics::Aerodynamics::VoxelWave(_)
                | crate::aerodynamics::Aerodynamics::Colliders
        )
        .then(|| std::sync::Arc::new(()));
        self.aerodynamics = aerodynamics;
    }

    /// Moves primary configuration into a part without cloning owned geometry.
    pub(crate) fn take_primary_aerodynamics(&mut self) -> crate::aerodynamics::Aerodynamics {
        self.aerodynamic_geometry_identity = None;
        std::mem::take(&mut self.aerodynamics)
    }

    pub fn aerodynamic_surfaces(&self) -> &[crate::aerodynamics::AerodynamicSurface] {
        &self.aerodynamic_surfaces
    }

    /// Opts into ideal, isolated, non-rotating sphere pressure for the current
    /// Mesh or Colliders geometry. This inviscid approximation has no wake or
    /// viscous drag and is limited to Mach <= 0.2; it is not a rocket model.
    /// Center and radius are body-local. Geometry is checked at each world step.
    /// The caller must ensure isolation and a uniform flow; other nearby bodies
    /// and additional loads are not checked for aerodynamic interaction.
    /// Changing the primary aerodynamic configuration clears this setting.
    pub fn set_potential_sphere_pressure(
        &mut self,
        center: Vec3,
        radius: f64,
    ) -> Result<(), crate::Error> {
        use crate::aerodynamics::{Aerodynamics, pressure::PressureError};
        if !center.is_finite()
            || !radius.is_finite()
            || radius <= 0.0
            || !matches!(
                self.aerodynamics,
                Aerodynamics::Mesh(_) | Aerodynamics::Colliders
            )
        {
            return Err(crate::Error::from(PressureError::InvalidPatch));
        }
        self.potential_sphere = Some((center, radius));
        Ok(())
    }

    /// Restores the ordinary primary geometry pressure model.
    pub fn clear_potential_sphere_pressure(&mut self) {
        self.potential_sphere = None;
    }

    /// Replaces explicitly selected body-local base patches. The caller must
    /// No wake, overlap or shielding calculation is performed. Nonempty
    /// configurations set by this method are rejected by world steps; use the
    /// index-based mesh or collider selection APIs for simulation instead.
    /// These loads are currently supported only on non-cluster Mesh/Colliders bodies;
    /// cluster conversion rejects nonempty configurations without changing the body.
    /// For Colliders, adding or removing a collider, requesting mutable access,
    /// or changing tessellation clears patches; configure them after such edits.
    pub fn set_base_pressure_patches(
        &mut self,
        patches: Vec<(
            crate::aerodynamics::pressure::PressurePatch,
            crate::aerodynamics::pressure::BasePressure,
        )>,
    ) {
        self.base_pressure_patches = patches;
        self.base_pressure_triangles = None;
    }

    /// Returns the explicit body-local base-pressure configuration.
    pub fn base_pressure_patches(
        &self,
    ) -> &[(
        crate::aerodynamics::pressure::PressurePatch,
        crate::aerodynamics::pressure::BasePressure,
    )] {
        &self.base_pressure_patches
    }

    /// Selects whole triangles of the current primary Mesh as base patches.
    /// Indices must be unique and refer to outward-wound, exposed base faces.
    /// This verifies geometry membership, not shielding or wake applicability.
    /// On error the previous configuration is preserved. Collider-derived meshes
    /// and clipped/partially exposed faces are not supported by this method.
    pub fn set_mesh_base_triangles(
        &mut self,
        indices: &[usize],
        base: crate::aerodynamics::pressure::BasePressure,
    ) -> Result<(), crate::Error> {
        use crate::aerodynamics::{Aerodynamics, pressure::PressureError};
        let Aerodynamics::Mesh(mesh) = &self.aerodynamics else {
            return Err(crate::Error::from(PressureError::InvalidPatch));
        };
        let patches = Self::base_triangles_from_mesh(mesh, indices, base)?;
        let triangles = indices
            .iter()
            .map(|&i| mesh.triangles()[i].map(|v| mesh.vertices()[v]))
            .collect();
        self.base_pressure_patches = patches;
        self.base_pressure_triangles = Some(triangles);
        Ok(())
    }

    pub(crate) fn base_triangles_from_mesh(
        mesh: &crate::shape::TriangleMesh,
        indices: &[usize],
        base: crate::aerodynamics::pressure::BasePressure,
    ) -> Result<
        Vec<(
            crate::aerodynamics::pressure::PressurePatch,
            crate::aerodynamics::pressure::BasePressure,
        )>,
        crate::Error,
    > {
        use crate::aerodynamics::pressure::{PressureError, PressurePatch};
        if indices.len() > mesh.triangles().len() {
            return Err(crate::Error::from(PressureError::InvalidPatch));
        }
        let mut sorted = indices.to_vec();
        sorted.sort_unstable();
        if sorted.windows(2).any(|pair| pair[0] == pair[1]) {
            return Err(crate::Error::from(PressureError::InvalidPatch));
        }
        let mut patches = Vec::with_capacity(indices.len());
        for &index in indices {
            let face = mesh
                .triangles()
                .get(index)
                .ok_or_else(|| crate::Error::from(PressureError::InvalidPatch))?;
            let [a, b, c] = face.map(|i| mesh.vertices()[i]);
            let cross = b
                .checked_sub(a)
                .and_then(|u| c.checked_sub(a).and_then(|v| u.checked_cross(v)))
                .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
            let length = cross
                .checked_length()
                .ok_or_else(|| crate::Error::from(PressureError::ArithmeticOverflow))?;
            let center = a / 3.0 + b / 3.0 + c / 3.0;
            patches.push((PressurePatch::new(center, cross, length * 0.5)?, base));
        }
        Ok(patches)
    }

    pub(crate) fn invalidate_collider_base_patches(&mut self) {
        if matches!(
            self.aerodynamics,
            crate::aerodynamics::Aerodynamics::Colliders
        ) {
            self.base_pressure_patches.clear();
            self.base_pressure_triangles = None;
        }
    }

    /// Copies only motion, mass, and force state; owned geometry is never cloned.
    pub fn motion(&self) -> BodyMotion {
        self.motion
    }

    pub(crate) fn set_motion(&mut self, motion: BodyMotion) {
        self.motion = motion;
    }
}

impl std::ops::Deref for RigidBody {
    type Target = BodyMotion;

    fn deref(&self) -> &BodyMotion {
        &self.motion
    }
}

impl std::ops::DerefMut for RigidBody {
    fn deref_mut(&mut self) -> &mut BodyMotion {
        &mut self.motion
    }
}

/// Copyable physical state used for transactional preparation and solver staging.
/// Aerodynamic configuration and geometry remain owned by [`RigidBody`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BodyMotion {
    state: BodyState,
    physics: BodyPhysics,
    forces: BodyForces,
    ccd_mode: CcdMode,
}

impl BodyMotion {
    /// Creates a stationary body with no mass properties.
    pub fn new_static(transform: Transform) -> Self {
        Self::new(BodyType::Static, transform, None)
    }

    /// Creates a stationary kinematic body with no mass properties.
    pub fn new_kinematic(transform: Transform) -> Self {
        Self::new(BodyType::Kinematic, transform, None)
    }

    /// Creates a stationary dynamic body with the supplied mass properties.
    pub fn new_dynamic(transform: Transform, mass: MassProperties) -> Result<Self, Error> {
        let body = Self::new(BodyType::Dynamic, transform, Some(mass));
        body.world_center_of_mass()?;
        Ok(body)
    }

    fn new(body_type: BodyType, transform: Transform, mass: Option<MassProperties>) -> Self {
        Self {
            state: BodyState {
                transform,
                linear_velocity: Vec3::ZERO,
                angular_velocity: Vec3::ZERO,
                sleeping: false,
            },
            physics: BodyPhysics { body_type, mass },
            forces: BodyForces {
                force: Vec3::ZERO,
                torque: Vec3::ZERO,
            },
            ccd_mode: CcdMode::Discrete,
        }
    }

    pub fn state(&self) -> &BodyState {
        &self.state
    }

    pub fn physics(&self) -> &BodyPhysics {
        &self.physics
    }

    pub fn forces(&self) -> &BodyForces {
        &self.forces
    }

    pub fn ccd_mode(&self) -> CcdMode {
        self.ccd_mode
    }

    pub fn set_ccd_mode(&mut self, mode: CcdMode) {
        self.ccd_mode = mode;
    }

    /// Replaces the world transform while preserving velocities.
    ///
    /// A changed transform wakes a sleeping dynamic body. The update is rejected
    /// if its world-space center of mass overflows.
    pub fn set_transform(&mut self, transform: Transform) -> Result<(), Error> {
        world_center_of_mass_for(transform, self.physics.mass)?;

        if transform != self.state.transform {
            self.state.transform = transform;
            if self.physics.body_type == BodyType::Dynamic {
                self.state.sleeping = false;
            }
        }
        Ok(())
    }

    /// Sets the world-space linear velocity of the center of mass, in m/s.
    pub fn set_linear_velocity(&mut self, velocity: Vec3) -> Result<(), Error> {
        self.ensure_velocity_is_allowed()?;
        if !velocity.is_finite() {
            return Err(Error::from(BodyError::NonFiniteValue));
        }

        self.state.linear_velocity = velocity;
        if self.physics.body_type == BodyType::Dynamic && velocity != Vec3::ZERO {
            self.state.sleeping = false;
        }
        Ok(())
    }

    /// Sets the world-space angular velocity, in rad/s.
    pub fn set_angular_velocity(&mut self, velocity: Vec3) -> Result<(), Error> {
        self.ensure_velocity_is_allowed()?;
        if !velocity.is_finite() {
            return Err(Error::from(BodyError::NonFiniteValue));
        }

        self.state.angular_velocity = velocity;
        if self.physics.body_type == BodyType::Dynamic && velocity != Vec3::ZERO {
            self.state.sleeping = false;
        }
        Ok(())
    }

    /// Replaces dynamic mass properties without changing position or velocity.
    ///
    /// This is a state edit, not a model of fuel ejection, mass transfer, or
    /// attachment: it does not conserve linear or angular momentum. When the
    /// local center of mass changes, the stored linear velocity is retained
    /// as the velocity of the new center of mass. Callers must account for
    /// any physical impulse or momentum exchange required by their model.
    ///
    /// Changed mass properties wake the body and clear accumulated forces and
    /// torques, whose torque reference may otherwise be stale. Separately
    /// registered force sources are not removed by this operation.
    pub fn set_mass(&mut self, mass: MassProperties) -> Result<(), Error> {
        if self.physics.body_type != BodyType::Dynamic {
            return Err(Error::from(BodyError::BodyMustBeDynamic));
        }
        world_center_of_mass_for(self.state.transform, Some(mass))?;
        if self.physics.mass != Some(mass) {
            self.physics.mass = Some(mass);
            self.state.sleeping = false;
            self.forces.force = Vec3::ZERO;
            self.forces.torque = Vec3::ZERO;
        }
        Ok(())
    }

    /// Returns the center of mass in world space, or `None` for bodies without mass.
    pub fn world_center_of_mass(&self) -> Result<Option<Vec3>, Error> {
        world_center_of_mass_for(self.state.transform, self.physics.mass)
    }

    /// Puts a dynamic body to sleep, clearing its velocities and accumulated forces.
    pub fn sleep(&mut self) -> Result<(), Error> {
        if self.physics.body_type != BodyType::Dynamic {
            return Err(Error::from(BodyError::BodyMustBeDynamic));
        }

        self.state.sleeping = true;
        self.state.linear_velocity = Vec3::ZERO;
        self.state.angular_velocity = Vec3::ZERO;
        self.forces.force = Vec3::ZERO;
        self.forces.torque = Vec3::ZERO;
        Ok(())
    }

    /// Wakes a dynamic body. Other body types are unchanged.
    pub fn wake_up(&mut self) {
        if self.physics.body_type == BodyType::Dynamic {
            self.state.sleeping = false;
        }
    }

    fn ensure_velocity_is_allowed(&self) -> Result<(), Error> {
        if self.physics.body_type == BodyType::Static {
            Err(Error::from(BodyError::StaticBodyCannotHaveVelocity))
        } else {
            Ok(())
        }
    }

    /// Accumulates a world-space force in newtons without changing velocity immediately.
    pub fn apply_force(&mut self, force: Vec3) -> Result<(), Error> {
        self.ensure_dynamic()?;
        if !force.is_finite() {
            return Err(Error::from(BodyError::NonFiniteValue));
        }

        let next_force = add_checked(self.forces.force, force)?;
        self.forces.force = next_force;
        if force != Vec3::ZERO {
            self.state.sleeping = false;
        }
        Ok(())
    }

    /// Accumulates force in newtons and torque about the center of mass in N·m.
    ///
    /// `point` is expressed in world-space meters.
    pub fn apply_force_at_point(&mut self, force: Vec3, point: Vec3) -> Result<(), Error> {
        self.ensure_dynamic()?;
        if !force.is_finite() || !point.is_finite() {
            return Err(Error::from(BodyError::NonFiniteValue));
        }

        let center = world_center_of_mass_for(self.state.transform, self.physics.mass)?
            .ok_or(BodyError::BodyMustBeDynamic)?;
        let torque_delta = point
            .checked_sub(center)
            .and_then(|offset| offset.checked_cross(force))
            .ok_or_else(|| Error::from(BodyError::ArithmeticOverflow))?;
        let next_force = add_checked(self.forces.force, force)?;
        let next_torque = add_checked(self.forces.torque, torque_delta)?;

        self.forces.force = next_force;
        self.forces.torque = next_torque;
        if force != Vec3::ZERO {
            self.state.sleeping = false;
        }
        Ok(())
    }

    /// Accumulates a world-space torque about the center of mass, in N·m.
    pub fn apply_torque(&mut self, torque: Vec3) -> Result<(), Error> {
        self.ensure_dynamic()?;
        if !torque.is_finite() {
            return Err(Error::from(BodyError::NonFiniteValue));
        }

        let next_torque = add_checked(self.forces.torque, torque)?;
        self.forces.torque = next_torque;
        if torque != Vec3::ZERO {
            self.state.sleeping = false;
        }
        Ok(())
    }

    /// Clears accumulated forces and torques without changing motion or sleep state.
    pub fn clear_forces(&mut self) {
        self.forces.force = Vec3::ZERO;
        self.forces.torque = Vec3::ZERO;
    }

    pub(crate) fn commit_integrated_state(&mut self, state: BodyState) {
        self.state = state;
        self.clear_forces();
    }

    pub(crate) fn set_state_preserving_forces(&mut self, state: BodyState) {
        self.state = state;
    }

    fn ensure_dynamic(&self) -> Result<(), Error> {
        if self.physics.body_type == BodyType::Dynamic {
            Ok(())
        } else {
            Err(Error::from(BodyError::BodyMustBeDynamic))
        }
    }

    /// Applies a world-space impulse immediately to the center of mass, in kg·m/s.
    pub fn apply_impulse(&mut self, impulse: Vec3) -> Result<(), Error> {
        self.ensure_dynamic()?;
        if !impulse.is_finite() {
            return Err(Error::from(BodyError::NonFiniteValue));
        }
        if impulse == Vec3::ZERO {
            return Ok(());
        }

        let mass = self.physics.mass.ok_or(BodyError::BodyMustBeDynamic)?;
        let delta_velocity = impulse / mass.mass();
        let next_velocity = add_checked(self.state.linear_velocity, delta_velocity)?;

        self.state.linear_velocity = next_velocity;
        self.state.sleeping = false;
        Ok(())
    }

    /// Applies a world-space impulse in kg·m/s at a world-space point in meters.
    pub fn apply_impulse_at_point(&mut self, impulse: Vec3, point: Vec3) -> Result<(), Error> {
        self.ensure_dynamic()?;
        if !impulse.is_finite() || !point.is_finite() {
            return Err(Error::from(BodyError::NonFiniteValue));
        }
        if impulse == Vec3::ZERO {
            return Ok(());
        }

        let mass = self.physics.mass.ok_or(BodyError::BodyMustBeDynamic)?;
        let center = world_center_of_mass_for(self.state.transform, Some(mass))?
            .ok_or(BodyError::BodyMustBeDynamic)?;
        let angular_impulse = point
            .checked_sub(center)
            .and_then(|offset| offset.checked_cross(impulse))
            .ok_or_else(|| Error::from(BodyError::ArithmeticOverflow))?;
        self.apply_constraint_impulse(impulse, angular_impulse)
    }

    pub(crate) fn apply_constraint_impulse(
        &mut self,
        linear_impulse: Vec3,
        angular_impulse: Vec3,
    ) -> Result<(), Error> {
        self.ensure_dynamic()?;
        if !linear_impulse.is_finite() || !angular_impulse.is_finite() {
            return Err(Error::from(BodyError::NonFiniteValue));
        }
        if linear_impulse == Vec3::ZERO && angular_impulse == Vec3::ZERO {
            return Ok(());
        }

        let mass = self.physics.mass.ok_or(BodyError::BodyMustBeDynamic)?;
        let rotation = self.state.transform.rotation();
        let local_angular_impulse = rotation
            .conjugate()
            .rotate_vector(angular_impulse)
            .ok_or_else(|| Error::from(BodyError::ArithmeticOverflow))?;
        let local_delta_angular_velocity = solve_inertia(mass.inertia(), local_angular_impulse)?;
        let delta_angular_velocity = rotation
            .rotate_vector(local_delta_angular_velocity)
            .ok_or_else(|| Error::from(BodyError::ArithmeticOverflow))?;
        let delta_linear_velocity = linear_impulse / mass.mass();
        let next_linear_velocity = add_checked(self.state.linear_velocity, delta_linear_velocity)?;
        let next_angular_velocity =
            add_checked(self.state.angular_velocity, delta_angular_velocity)?;

        self.state.linear_velocity = next_linear_velocity;
        self.state.angular_velocity = next_angular_velocity;
        self.state.sleeping = false;
        Ok(())
    }

    pub(crate) fn apply_position_impulse(
        &mut self,
        impulse: Vec3,
        point: Vec3,
    ) -> Result<(), Error> {
        if self.physics.body_type != BodyType::Dynamic {
            return Ok(());
        }
        if !impulse.is_finite() || !point.is_finite() {
            return Err(Error::from(BodyError::NonFiniteValue));
        }
        if impulse == Vec3::ZERO {
            return Ok(());
        }

        let mass = self.physics.mass.ok_or(BodyError::BodyMustBeDynamic)?;
        let center = world_center_of_mass_for(self.state.transform, Some(mass))?
            .ok_or(BodyError::BodyMustBeDynamic)?;
        let angular_impulse = point
            .checked_sub(center)
            .and_then(|offset| offset.checked_cross(impulse))
            .ok_or_else(|| Error::from(BodyError::ArithmeticOverflow))?;
        self.apply_constraint_position_impulse(impulse, angular_impulse)
    }

    pub(crate) fn apply_constraint_position_impulse(
        &mut self,
        linear_impulse: Vec3,
        angular_impulse: Vec3,
    ) -> Result<(), Error> {
        if self.physics.body_type != BodyType::Dynamic {
            return Ok(());
        }
        if !linear_impulse.is_finite() || !angular_impulse.is_finite() {
            return Err(Error::from(BodyError::NonFiniteValue));
        }
        if linear_impulse == Vec3::ZERO && angular_impulse == Vec3::ZERO {
            return Ok(());
        }

        let mass = self.physics.mass.ok_or(BodyError::BodyMustBeDynamic)?;
        let center = world_center_of_mass_for(self.state.transform, Some(mass))?
            .ok_or(BodyError::BodyMustBeDynamic)?;
        let rotation = self.state.transform.rotation();
        let local_angular_impulse = rotation
            .conjugate()
            .rotate_vector(angular_impulse)
            .ok_or_else(|| Error::from(BodyError::ArithmeticOverflow))?;
        let local_rotation_delta = solve_inertia(mass.inertia(), local_angular_impulse)?;
        let rotation_delta = rotation
            .rotate_vector(local_rotation_delta)
            .ok_or_else(|| Error::from(BodyError::ArithmeticOverflow))?;
        let angle = rotation_delta
            .checked_length()
            .ok_or_else(|| Error::from(BodyError::ArithmeticOverflow))?;
        let rotation_delta = if angle > 0.0 {
            Quat::from_axis_angle(rotation_delta / angle, angle)
                .ok_or_else(|| Error::from(BodyError::ArithmeticOverflow))?
        } else {
            Quat::IDENTITY
        };
        let next_rotation = (rotation_delta * rotation)
            .try_normalize()
            .filter(|rotation| rotation.is_normalized())
            .ok_or_else(|| Error::from(BodyError::ArithmeticOverflow))?;
        let next_center = center
            .checked_add(linear_impulse / mass.mass())
            .ok_or_else(|| Error::from(BodyError::ArithmeticOverflow))?;
        let next_translation = next_center
            .checked_sub(
                next_rotation
                    .rotate_vector(mass.center_of_mass())
                    .ok_or_else(|| Error::from(BodyError::ArithmeticOverflow))?,
            )
            .ok_or_else(|| Error::from(BodyError::ArithmeticOverflow))?;
        let next_transform = Transform::new(next_translation, next_rotation)
            .map_err(|_| BodyError::ArithmeticOverflow)?;
        self.set_position_projected_transform(next_transform, None)
    }

    /// A geometric orientation correction carries no physical angular impulse.
    /// Keep spin momentum fixed while resolving angular velocity in the new inertia.
    fn set_position_projected_transform(
        &mut self,
        transform: Transform,
        inverse_inertia: Option<kapitsa_math::Mat3>,
    ) -> Result<(), Error> {
        let rotation = self.state.transform.rotation();
        let next_rotation = transform.rotation();
        let omega = self.state.angular_velocity;
        if rotation == next_rotation || omega == Vec3::ZERO {
            return self.set_transform(transform);
        }
        let mass = self.physics.mass.ok_or(BodyError::BodyMustBeDynamic)?;
        let local_omega = rotation
            .conjugate()
            .rotate_vector(omega)
            .ok_or_else(|| Error::from(BodyError::ArithmeticOverflow))?;
        let momentum = mass
            .inertia()
            .checked_mul_vector(local_omega)
            .and_then(|value| rotation.rotate_vector(value))
            .ok_or_else(|| Error::from(BodyError::ArithmeticOverflow))?;
        let next_omega = if let Some(inverse_inertia) = inverse_inertia {
            // Rotate the cached response with the pose instead of factoring
            // the same local inertia again: I_new^-1 = Q I_old^-1 Q^T.
            let delta = next_rotation * rotation.conjugate();
            delta
                .conjugate()
                .rotate_vector(momentum)
                .and_then(|value| inverse_inertia.checked_mul_vector(value))
                .and_then(|value| delta.rotate_vector(value))
                .ok_or_else(|| Error::from(BodyError::ArithmeticOverflow))?
        } else {
            let next_local_momentum = next_rotation
                .conjugate()
                .rotate_vector(momentum)
                .ok_or_else(|| Error::from(BodyError::ArithmeticOverflow))?;
            next_rotation
                .rotate_vector(solve_inertia(mass.inertia(), next_local_momentum)?)
                .ok_or_else(|| Error::from(BodyError::ArithmeticOverflow))?
        };
        self.set_transform(transform)?;
        self.state.angular_velocity = next_omega;
        Ok(())
    }
}

fn world_center_of_mass_for(
    transform: Transform,
    mass: Option<MassProperties>,
) -> Result<Option<Vec3>, Error> {
    let Some(mass) = mass else {
        return Ok(None);
    };

    let center = transform
        .rotation()
        .rotate_vector(mass.center_of_mass())
        .and_then(|center| center.checked_add(transform.translation()))
        .ok_or_else(|| Error::from(BodyError::ArithmeticOverflow))?;
    Ok(Some(center))
}

fn solve_inertia(inertia: kapitsa_math::Mat3, rhs: Vec3) -> Result<Vec3, Error> {
    match inertia.solve_spd(rhs) {
        Ok(solution) => Ok(solution),
        Err(CholeskyError::NotSymmetric | CholeskyError::NotPositiveDefinite) => {
            Err(Error::from(BodyError::InvalidInertiaTensor))
        }
        Err(CholeskyError::NonFiniteInput) => Err(Error::from(BodyError::NonFiniteValue)),
        Err(CholeskyError::ResultOutOfRange) => Err(Error::from(BodyError::ArithmeticOverflow)),
    }
}

fn add_checked(current: Vec3, delta: Vec3) -> Result<Vec3, Error> {
    current
        .checked_add(delta)
        .ok_or_else(|| Error::from(BodyError::ArithmeticOverflow))
}

#[cfg(test)]
#[path = "rigid_body_tests.rs"]
mod tests;
