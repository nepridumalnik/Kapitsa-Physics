//! Momentum-form constrained integration for a single free, hard Ball pair.
//!
//! Only the two paired kicks change momenta. Each acts at one common world
//! point, so both total linear and angular momentum are preserved. The start
//! kick is solved against the *drifted* anchors; no position projection follows.

use kapitsa_math::{Mat3, Quat, Vec3};

use crate::{
    collision::CollisionError,
    mass::MassProperties,
    rigid_body::{BodyState, RigidBody, solver_mass::inverse_spd},
    transform::Transform,
};

use super::solver::map_rigid_body_error;

const POSITION_TOLERANCE: f64 = 1.0e-13;
const VELOCITY_TOLERANCE: f64 = 1.0e-12;
const MAX_ITERATIONS: usize = 32;

#[derive(Clone, Copy)]
struct Body {
    center: Vec3,
    rotation: Quat,
    momentum: Vec3,
    spin: Vec3,
    mass: MassProperties,
    inverse_inertia: Mat3,
    anchor: Vec3,
}

impl Body {
    fn new(body: &RigidBody, anchor: Vec3) -> Result<Self, CollisionError> {
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
            anchor: anchor - mass.center_of_mass(),
        })
    }

    fn anchor(self) -> Result<Vec3, CollisionError> {
        finite(self.center + rotate(self.rotation, self.anchor)?)
    }

    fn inverse_angular_mass(self, spin: Vec3) -> Result<Vec3, CollisionError> {
        rotate(
            self.rotation,
            self.inverse_inertia * rotate(self.rotation.conjugate(), spin)?,
        )
    }

    fn omega(self) -> Result<Vec3, CollisionError> {
        self.inverse_angular_mass(self.spin)
    }

    fn anchor_velocity(self) -> Result<Vec3, CollisionError> {
        finite(
            self.momentum / self.mass.mass()
                + self.omega()?.cross(rotate(self.rotation, self.anchor)?),
        )
    }

    fn kick(&mut self, impulse: Vec3, point: Vec3) -> Result<(), CollisionError> {
        self.momentum = finite(self.momentum + impulse)?;
        self.spin = finite(self.spin + (point - self.center).cross(impulse))?;
        Ok(())
    }

    fn drift(&mut self, dt: f64) -> Result<(), CollisionError> {
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

    fn state(self, mut body: RigidBody) -> Result<BodyState, CollisionError> {
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

/// The caller admits only an awake, force-free dynamic pair with a hard Ball.
/// All work stays on the stack; failure leaves the source bodies untouched.
pub(crate) fn predict(
    bodies: [&RigidBody; 2],
    anchors: [Vec3; 2],
    dt: f64,
) -> Result<[BodyState; 2], CollisionError> {
    if !dt.is_finite() || dt <= 0.0 {
        return Err(CollisionError::InvalidConfig);
    }
    let start = [
        Body::new(bodies[0], anchors[0])?,
        Body::new(bodies[1], anchors[1])?,
    ];
    let point = common_anchor(start)?;
    // Analytic start effective mass is a fixed quasi-Newton approximation to
    // (1/dt) d(anchor_b(end)-anchor_a(end))/d(start_impulse). Unlike finite
    // differences, every iteration needs only one constrained drift.
    let response = effective_mass(start, point)?
        .factor_spd()
        .map_err(|_| CollisionError::NonConvergent)?;
    let mut impulse = Vec3::ZERO;
    for _ in 0..MAX_ITERATIONS {
        let mut next = start;
        kick_pair(&mut next, impulse, point)?;
        for body in &mut next {
            body.drift(dt)?;
        }
        let error = finite(next[1].anchor()? - next[0].anchor()?)?;
        if length(error)? <= POSITION_TOLERANCE {
            finish_velocity(&mut next)?;
            return Ok([next[0].state(*bodies[0])?, next[1].state(*bodies[1])?]);
        }
        let correction = response
            .solve(error / dt)
            .map_err(|_| CollisionError::ArithmeticOverflow)?;
        impulse = finite(impulse - correction)?;
    }
    Err(CollisionError::NonConvergent)
}

fn finish_velocity(bodies: &mut [Body; 2]) -> Result<(), CollisionError> {
    let point = common_anchor(*bodies)?;
    let response = effective_mass(*bodies, point)?
        .factor_spd()
        .map_err(|_| CollisionError::NonConvergent)?;
    // The anchors differ by at most the position tolerance; use their actual
    // velocities and validate the residual after applying the common-point kick.
    for _ in 0..3 {
        let relative = finite(bodies[1].anchor_velocity()? - bodies[0].anchor_velocity()?)?;
        if length(relative)? <= VELOCITY_TOLERANCE {
            return Ok(());
        }
        let impulse = response
            .solve(-relative)
            .map_err(|_| CollisionError::ArithmeticOverflow)?;
        kick_pair(bodies, impulse, point)?;
    }
    Err(CollisionError::NonConvergent)
}

fn common_anchor(bodies: [Body; 2]) -> Result<Vec3, CollisionError> {
    finite(bodies[0].anchor()? * 0.5 + bodies[1].anchor()? * 0.5)
}

fn kick_pair(bodies: &mut [Body; 2], impulse: Vec3, point: Vec3) -> Result<(), CollisionError> {
    bodies[0].kick(-impulse, point)?;
    bodies[1].kick(impulse, point)
}

fn effective_mass(bodies: [Body; 2], point: Vec3) -> Result<Mat3, CollisionError> {
    let mut columns = [Vec3::ZERO; 3];
    for (column, axis) in columns.iter_mut().zip([Vec3::X, Vec3::Y, Vec3::Z]) {
        for body in bodies {
            let arm = finite(point - body.center)?;
            *column +=
                axis / body.mass.mass() + body.inverse_angular_mass(arm.cross(axis))?.cross(arm);
        }
        finite(*column)?;
    }
    // Suppress only roundoff asymmetry before the SPD factorization.
    let matrix = Mat3::from_cols(columns[0], columns[1], columns[2]);
    let matrix = (matrix + matrix.transpose()) * 0.5;
    if !matrix.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }
    Ok(matrix)
}

fn rotate(rotation: Quat, vector: Vec3) -> Result<Vec3, CollisionError> {
    rotation
        .rotate_vector(vector)
        .ok_or(CollisionError::ArithmeticOverflow)
}

fn finite(vector: Vec3) -> Result<Vec3, CollisionError> {
    vector
        .is_finite()
        .then_some(vector)
        .ok_or(CollisionError::ArithmeticOverflow)
}

fn length(vector: Vec3) -> Result<f64, CollisionError> {
    vector
        .checked_length()
        .ok_or(CollisionError::ArithmeticOverflow)
}
