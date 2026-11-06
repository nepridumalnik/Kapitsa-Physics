use super::{finite, length, rotate};
use crate::joint::solver::map_rigid_body_error;
use crate::{
    collision::CollisionError,
    mass::MassProperties,
    rigid_body::{BodyState, RigidBody, solver_mass::inverse_spd},
    transform::Transform,
};
use kapitsa_math::{Mat3, Quat, Vec3};
const MAX_ITERATIONS: usize = 32;

#[derive(Clone, Copy)]
pub(crate) struct MomentumBody {
    pub(crate) center: Vec3,
    pub(crate) rotation: Quat,
    pub(crate) momentum: Vec3,
    pub(crate) spin: Vec3,
    pub(crate) mass: MassProperties,
    pub(crate) inverse_inertia: Mat3,
}

impl MomentumBody {
    pub(crate) fn new(body: &RigidBody) -> Result<Self, CollisionError> {
        let mass = body
            .physics()
            .mass()
            .ok_or(CollisionError::InvalidContact)?;
        let state = body.state();
        let rotation = state.transform().rotation();
        let local_omega = rotate(rotation.conjugate(), state.angular_velocity())?;
        let spin = rotate(rotation, mass.inertia() * local_omega)?;
        let momentum = state.linear_velocity() * mass.mass();
        finite(momentum)?;
        Ok(Self {
            center: body
                .world_center_of_mass()
                .map_err(map_rigid_body_error)?
                .ok_or(CollisionError::InvalidContact)?,
            rotation,
            momentum,
            spin,
            mass,
            inverse_inertia: inverse_spd(mass.inertia())?,
        })
    }

    pub(crate) fn inverse_angular_mass(self, spin: Vec3) -> Result<Vec3, CollisionError> {
        rotate(
            self.rotation,
            self.inverse_inertia * rotate(self.rotation.conjugate(), spin)?,
        )
    }

    pub(crate) fn omega(self) -> Result<Vec3, CollisionError> {
        self.inverse_angular_mass(self.spin)
    }

    pub(crate) fn kick(&mut self, linear: Vec3, angular: Vec3) -> Result<(), CollisionError> {
        self.momentum = finite(self.momentum + linear)?;
        self.spin = finite(self.spin + angular)?;
        Ok(())
    }

    pub(crate) fn anchor(self, local: Vec3) -> Result<Vec3, CollisionError> {
        finite(self.center + rotate(self.rotation, local - self.mass.center_of_mass())?)
    }

    pub(crate) fn drift(&mut self, dt: f64) -> Result<(), CollisionError> {
        self.center = finite(self.center + self.momentum * (dt / self.mass.mass()))?;
        let rotation = self.rotation;
        let mut omega = self.omega()?;
        // Implicit rotational midpoint with constant world spin momentum.
        for _ in 0..MAX_ITERATIONS {
            self.rotation = rotation
                .integrate_world_angular_velocity(omega, dt * 0.5)
                .ok_or(CollisionError::ArithmeticOverflow)?;
            let next = self.omega()?;
            let residual = length(next - omega)?;
            omega = next;
            if residual <= 1.0e-14 * (1.0 + length(omega)?) {
                self.rotation = rotation
                    .integrate_world_angular_velocity(omega, dt)
                    .ok_or(CollisionError::ArithmeticOverflow)?;
                return Ok(());
            }
        }
        Err(CollisionError::NonConvergent)
    }

    pub(crate) fn state(self, mut body: RigidBody) -> Result<BodyState, CollisionError> {
        let origin = finite(self.center - rotate(self.rotation, self.mass.center_of_mass())?)?;
        let transform = Transform::new(origin, self.rotation)
            .map_err(|_| CollisionError::ArithmeticOverflow)?;
        body.set_transform(transform)
            .map_err(map_rigid_body_error)?;
        body.set_linear_velocity(self.momentum / self.mass.mass())
            .map_err(map_rigid_body_error)?;
        body.set_angular_velocity(self.omega()?)
            .map_err(map_rigid_body_error)?;
        Ok(*body.state())
    }
}
