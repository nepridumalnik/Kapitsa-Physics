use crate::{
    constraint,
    joint::{JointConstraint, JointSolver, JointWorkspace},
    rigid_body::RigidBody,
};
use kapitsa_math::Vec3;

use super::{
    CollisionConfig, CollisionError,
    manifold::{ContactManifold, ContactPoint},
};

const POSITION_CORRECTION_FRACTION: f64 = 0.2;

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct SolverMaterial {
    pub(crate) restitution: f64,
    pub(crate) static_friction: f64,
    pub(crate) dynamic_friction: f64,
}

pub(crate) struct SolverBody {
    body: RigidBody,
}

impl SolverBody {
    pub(crate) fn state(&self) -> crate::rigid_body::BodyState {
        *self.body.state()
    }

    pub(crate) fn from_prediction(
        mut body: RigidBody,
        state: crate::rigid_body::BodyState,
    ) -> Self {
        body.commit_integrated_state(state);
        Self { body }
    }

    pub(crate) fn body(&self) -> &RigidBody {
        &self.body
    }

    pub(crate) fn body_mut(&mut self) -> &mut RigidBody {
        &mut self.body
    }
}

#[derive(Clone, Copy)]
pub(crate) struct SolverManifold {
    pub(crate) body_a: usize,
    pub(crate) body_b: usize,
    pub(crate) manifold: ContactManifold,
    pub(crate) material: SolverMaterial,
}

#[derive(Clone, Copy)]
struct SolverPoint {
    manifold: usize,
    point: usize,
    body_a: usize,
    body_b: usize,
    position_a: Vec3,
    position_b: Vec3,
    normal: Vec3,
    tangent_a: Vec3,
    tangent_b: Vec3,
    restitution_target: f64,
}

#[derive(Default)]
pub(crate) struct SolverWorkspace {
    points: Vec<SolverPoint>,
}

pub(crate) struct SolverIsland {
    pub(crate) body_indices: Vec<usize>,
    pub(crate) manifold_indices: Vec<usize>,
    pub(crate) joint_indices: Vec<usize>,
    pub(crate) bodies: Vec<SolverBody>,
    pub(crate) manifolds: Vec<SolverManifold>,
    pub(crate) joints: Vec<JointConstraint>,
    pub(crate) workspace: SolverWorkspace,
    pub(crate) joint_workspace: JointWorkspace,
    pub(crate) worker_error: Option<CollisionError>,
}

impl SolverIsland {
    pub(crate) const fn new() -> Self {
        Self {
            body_indices: Vec::new(),
            manifold_indices: Vec::new(),
            joint_indices: Vec::new(),
            bodies: Vec::new(),
            manifolds: Vec::new(),
            joints: Vec::new(),
            workspace: SolverWorkspace::new(),
            joint_workspace: JointWorkspace::new(),
            worker_error: None,
        }
    }

    pub(crate) fn clear(&mut self) {
        self.body_indices.clear();
        self.manifold_indices.clear();
        self.joint_indices.clear();
        self.bodies.clear();
        self.manifolds.clear();
        self.joints.clear();
        self.joint_workspace.clear();
        self.worker_error = None;
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn retained_capacity_bytes(&self) -> usize {
        use std::mem::size_of;

        self.body_indices.capacity() * size_of::<usize>()
            + self.manifold_indices.capacity() * size_of::<usize>()
            + self.joint_indices.capacity() * size_of::<usize>()
            + self.bodies.capacity() * size_of::<SolverBody>()
            + self.manifolds.capacity() * size_of::<SolverManifold>()
            + self.joints.capacity() * size_of::<JointConstraint>()
            + self.workspace.retained_capacity_bytes()
            + self.joint_workspace.retained_capacity_bytes()
    }
}

impl SolverWorkspace {
    pub(crate) const fn new() -> Self {
        Self { points: Vec::new() }
    }

    pub(crate) fn clear(&mut self) {
        self.points.clear();
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn retained_capacity_bytes(&self) -> usize {
        self.points.capacity() * std::mem::size_of::<SolverPoint>()
    }

    pub(crate) fn solve(
        &mut self,
        bodies: &mut [SolverBody],
        manifolds: &mut [SolverManifold],
        config: CollisionConfig,
    ) -> Result<(), CollisionError> {
        config.validate()?;
        self.prepare_points(bodies, manifolds, config)?;
        self.warm_start(bodies, manifolds)?;
        for _ in 0..config.velocity_iterations {
            self.solve_velocity_iteration(bodies, manifolds)?;
        }
        self.solve_position_constraints(bodies, manifolds, config)
    }

    pub(crate) fn solve_with_joints(
        &mut self,
        bodies: &mut [SolverBody],
        manifolds: &mut [SolverManifold],
        joint_workspace: &mut JointWorkspace,
        joints: &[JointConstraint],
        dt: f64,
        config: CollisionConfig,
    ) -> Result<(), CollisionError> {
        if joints.is_empty() {
            return self.solve(bodies, manifolds, config);
        }
        config.validate()?;
        if !dt.is_finite() || dt <= 0.0 {
            return Err(CollisionError::InvalidConfig);
        }
        self.prepare_points(bodies, manifolds, config)?;
        self.warm_start(bodies, manifolds)?;
        JointSolver::begin_step(joint_workspace);

        for _ in 0..config.velocity_iterations {
            self.solve_velocity_iteration(bodies, manifolds)?;
            JointSolver::prepare(joint_workspace, bodies, joints, dt)?;
            JointSolver::solve_velocity_iteration(joint_workspace, bodies, dt)?;
        }
        for _ in 0..config.position_iterations {
            self.solve_position_iteration(bodies, manifolds, config)?;
            JointSolver::prepare(joint_workspace, bodies, joints, dt)?;
            JointSolver::solve_position_iteration(joint_workspace, bodies, dt)?;
        }
        Ok(())
    }

    fn prepare_points(
        &mut self,
        bodies: &[SolverBody],
        manifolds: &[SolverManifold],
        config: CollisionConfig,
    ) -> Result<(), CollisionError> {
        self.points.clear();
        let count = manifolds.iter().try_fold(0_usize, |count, manifold| {
            count
                .checked_add(manifold.manifold.len())
                .ok_or(CollisionError::SpatialIndexCapacityExceeded)
        })?;
        self.points
            .try_reserve(count)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;

        for (manifold_index, manifold) in manifolds.iter().enumerate() {
            if manifold.body_a == manifold.body_b
                || manifold.body_a >= bodies.len()
                || manifold.body_b >= bodies.len()
            {
                return Err(CollisionError::InvalidContact);
            }
            validate_material(manifold.material)?;
            let body_a = bodies
                .get(manifold.body_a)
                .ok_or(CollisionError::InvalidContact)?;
            let body_b = bodies
                .get(manifold.body_b)
                .ok_or(CollisionError::InvalidContact)?;
            for (point_index, contact) in manifold.manifold.points().iter().enumerate() {
                validate_contact(*contact)?;
                let normal = contact
                    .normal
                    .try_normalize()
                    .ok_or(CollisionError::InvalidContact)?;
                let (tangent_a, tangent_b) = tangent_basis(normal)?;
                let position_a = body_a
                    .body
                    .state()
                    .transform()
                    .transform_point(contact.local_a);
                let position_b = body_b
                    .body
                    .state()
                    .transform()
                    .transform_point(contact.local_b);
                let relative_velocity =
                    relative_velocity(&body_a.body, &body_b.body, position_a, position_b)?;
                let normal_speed = relative_velocity
                    .checked_dot(normal)
                    .ok_or(CollisionError::ArithmeticOverflow)?;
                let restitution_target = if normal_speed <= -config.restitution_threshold {
                    manifold.material.restitution * -normal_speed
                } else {
                    0.0
                };
                if !position_a.is_finite()
                    || !position_b.is_finite()
                    || !restitution_target.is_finite()
                {
                    return Err(CollisionError::ArithmeticOverflow);
                }
                self.points.push(SolverPoint {
                    manifold: manifold_index,
                    point: point_index,
                    body_a: manifold.body_a,
                    body_b: manifold.body_b,
                    position_a,
                    position_b,
                    normal,
                    tangent_a,
                    tangent_b,
                    restitution_target,
                });
            }
        }
        Ok(())
    }

    fn warm_start(
        &self,
        bodies: &mut [SolverBody],
        manifolds: &[SolverManifold],
    ) -> Result<(), CollisionError> {
        for point in self.points.iter().copied() {
            let contact = contact_at(manifolds, point)?;
            let impulse = point.normal * contact.normal_impulse
                + point.tangent_a * contact.tangent_impulse[0]
                + point.tangent_b * contact.tangent_impulse[1];
            apply_pair_impulse(
                bodies,
                point.body_a,
                point.body_b,
                point.position_a,
                point.position_b,
                impulse,
            )?;
        }
        Ok(())
    }

    fn solve_velocity_iteration(
        &self,
        bodies: &mut [SolverBody],
        manifolds: &mut [SolverManifold],
    ) -> Result<(), CollisionError> {
        for point in self.points.iter().copied() {
            let contact = *contact_at(manifolds, point)?;
            let (body_a, body_b) = bodies_at(bodies, point.body_a, point.body_b)?;
            let denominator = effective_mass(
                &body_a.body,
                &body_b.body,
                point.position_a,
                point.position_b,
                point.normal,
            )?;
            if denominator <= f64::MIN_POSITIVE {
                return Err(CollisionError::InvalidContact);
            }
            let velocity = relative_velocity(
                &body_a.body,
                &body_b.body,
                point.position_a,
                point.position_b,
            )?;
            let normal_speed = velocity
                .checked_dot(point.normal)
                .ok_or(CollisionError::ArithmeticOverflow)?;
            let delta = (point.restitution_target - normal_speed) / denominator;
            if !delta.is_finite() {
                return Err(CollisionError::ArithmeticOverflow);
            }
            let next_normal_impulse = (contact.normal_impulse + delta).max(0.0);
            let normal_delta = next_normal_impulse - contact.normal_impulse;
            if normal_delta != 0.0 {
                apply_pair_impulse(
                    bodies,
                    point.body_a,
                    point.body_b,
                    point.position_a,
                    point.position_b,
                    point.normal * normal_delta,
                )?;
                let contact = contact_mut(manifolds, point)?;
                contact.normal_impulse = next_normal_impulse;
                if normal_delta > 0.0 {
                    let total_delta = contact.normal_impulse_delta + normal_delta;
                    if !total_delta.is_finite() {
                        return Err(CollisionError::ArithmeticOverflow);
                    }
                    contact.normal_impulse_delta = total_delta;
                }
            }

            let contact = *contact_at(manifolds, point)?;
            let material = manifolds
                .get(point.manifold)
                .ok_or(CollisionError::InvalidContact)?
                .material;
            let (body_a, body_b) = bodies_at(bodies, point.body_a, point.body_b)?;
            let velocity = relative_velocity(
                &body_a.body,
                &body_b.body,
                point.position_a,
                point.position_b,
            )?;
            let (tangent_impulse, delta_a) = solve_tangent_impulse(
                &body_a.body,
                &body_b.body,
                point,
                contact,
                material,
                velocity,
            )?;
            if delta_a != Vec3::ZERO {
                apply_pair_impulse(
                    bodies,
                    point.body_a,
                    point.body_b,
                    point.position_a,
                    point.position_b,
                    delta_a,
                )?;
            }
            contact_mut(manifolds, point)?.tangent_impulse = tangent_impulse;
        }
        Ok(())
    }

    fn solve_position_constraints(
        &self,
        bodies: &mut [SolverBody],
        manifolds: &[SolverManifold],
        config: CollisionConfig,
    ) -> Result<(), CollisionError> {
        for _ in 0..config.position_iterations {
            self.solve_position_iteration(bodies, manifolds, config)?;
        }
        Ok(())
    }

    fn solve_position_iteration(
        &self,
        bodies: &mut [SolverBody],
        manifolds: &[SolverManifold],
        config: CollisionConfig,
    ) -> Result<(), CollisionError> {
        for point in self.points.iter().copied() {
            let contact = contact_at(manifolds, point)?;
            let (body_a, body_b) = bodies_at(bodies, point.body_a, point.body_b)?;
            let position_a = body_a
                .body
                .state()
                .transform()
                .transform_point(contact.local_a);
            let position_b = body_b
                .body
                .state()
                .transform()
                .transform_point(contact.local_b);
            let separation = position_b
                .checked_sub(position_a)
                .and_then(|delta| delta.checked_dot(point.normal))
                .ok_or(CollisionError::ArithmeticOverflow)?;
            let correction =
                (-separation - config.penetration_slop).max(0.0) * POSITION_CORRECTION_FRACTION;
            if correction == 0.0 {
                continue;
            }
            let denominator = effective_mass(
                &body_a.body,
                &body_b.body,
                position_a,
                position_b,
                point.normal,
            )?;
            if denominator <= f64::MIN_POSITIVE {
                return Err(CollisionError::InvalidContact);
            }
            let impulse = point.normal * (correction / denominator);
            apply_pair_position_impulse(
                bodies,
                point.body_a,
                point.body_b,
                position_a,
                position_b,
                impulse,
            )?;
        }
        Ok(())
    }
}

fn solve_tangent_impulse(
    body_a: &RigidBody,
    body_b: &RigidBody,
    point: SolverPoint,
    contact: ContactPoint,
    material: SolverMaterial,
    velocity: Vec3,
) -> Result<([f64; 2], Vec3), CollisionError> {
    let k11 = effective_cross_mass(
        body_a,
        body_b,
        point.position_a,
        point.position_b,
        point.tangent_a,
        point.tangent_a,
    )?;
    let k12 = effective_cross_mass(
        body_a,
        body_b,
        point.position_a,
        point.position_b,
        point.tangent_a,
        point.tangent_b,
    )?;
    let k22 = effective_cross_mass(
        body_a,
        body_b,
        point.position_a,
        point.position_b,
        point.tangent_b,
        point.tangent_b,
    )?;
    let determinant = k11 * k22 - k12 * k12;
    if !determinant.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }
    if determinant <= f64::MIN_POSITIVE {
        return Ok((contact.tangent_impulse, Vec3::ZERO));
    }
    let speed_a = velocity
        .checked_dot(point.tangent_a)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let speed_b = velocity
        .checked_dot(point.tangent_b)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let delta_a = (-k22 * speed_a + k12 * speed_b) / determinant;
    let delta_b = (k12 * speed_a - k11 * speed_b) / determinant;
    let candidate = [
        contact.tangent_impulse[0] + delta_a,
        contact.tangent_impulse[1] + delta_b,
    ];
    if !candidate[0].is_finite() || !candidate[1].is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }
    let candidate_length = candidate[0].hypot(candidate[1]);
    let static_limit = material.static_friction * contact.normal_impulse;
    if !candidate_length.is_finite() || !static_limit.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }
    let (next_a, next_b) = if candidate_length <= static_limit {
        (candidate[0], candidate[1])
    } else if candidate_length > 0.0 {
        let dynamic_limit = material.dynamic_friction * contact.normal_impulse;
        if !dynamic_limit.is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
        let scale = dynamic_limit.min(candidate_length) / candidate_length;
        (candidate[0] * scale, candidate[1] * scale)
    } else {
        (0.0, 0.0)
    };
    let change_a = next_a - contact.tangent_impulse[0];
    let change_b = next_b - contact.tangent_impulse[1];
    let impulse = point.tangent_a * change_a + point.tangent_b * change_b;
    if !impulse.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }
    Ok(([next_a, next_b], impulse))
}

fn effective_mass(
    body_a: &RigidBody,
    body_b: &RigidBody,
    position_a: Vec3,
    position_b: Vec3,
    direction: Vec3,
) -> Result<f64, CollisionError> {
    effective_cross_mass(body_a, body_b, position_a, position_b, direction, direction)
}

fn effective_cross_mass(
    body_a: &RigidBody,
    body_b: &RigidBody,
    position_a: Vec3,
    position_b: Vec3,
    measurement: Vec3,
    impulse: Vec3,
) -> Result<f64, CollisionError> {
    constraint::effective_cross_mass_at(
        body_a,
        body_b,
        position_a,
        position_b,
        measurement,
        impulse,
    )
    .map_err(map_constraint_error)
}

fn relative_velocity(
    body_a: &RigidBody,
    body_b: &RigidBody,
    position_a: Vec3,
    position_b: Vec3,
) -> Result<Vec3, CollisionError> {
    constraint::relative_velocity_at(body_a, body_b, position_a, position_b)
        .map_err(map_constraint_error)
}

fn apply_pair_impulse(
    bodies: &mut [SolverBody],
    body_a: usize,
    body_b: usize,
    point_a: Vec3,
    point_b: Vec3,
    impulse: Vec3,
) -> Result<(), CollisionError> {
    let (a, b) = bodies_at_mut(bodies, body_a, body_b)?;
    constraint::apply_point_impulse_pair(&mut a.body, &mut b.body, point_a, point_b, impulse)
        .map_err(map_constraint_error)
}

fn apply_pair_position_impulse(
    bodies: &mut [SolverBody],
    body_a: usize,
    body_b: usize,
    point_a: Vec3,
    point_b: Vec3,
    impulse: Vec3,
) -> Result<(), CollisionError> {
    let (a, b) = bodies_at_mut(bodies, body_a, body_b)?;
    constraint::apply_point_position_impulse_pair(
        &mut a.body,
        &mut b.body,
        point_a,
        point_b,
        impulse,
    )
    .map_err(map_constraint_error)
}

fn bodies_at(
    bodies: &[SolverBody],
    body_a: usize,
    body_b: usize,
) -> Result<(&SolverBody, &SolverBody), CollisionError> {
    let a = bodies.get(body_a).ok_or(CollisionError::InvalidContact)?;
    let b = bodies.get(body_b).ok_or(CollisionError::InvalidContact)?;
    if body_a == body_b {
        return Err(CollisionError::InvalidContact);
    }
    Ok((a, b))
}

fn bodies_at_mut(
    bodies: &mut [SolverBody],
    body_a: usize,
    body_b: usize,
) -> Result<(&mut SolverBody, &mut SolverBody), CollisionError> {
    if body_a == body_b {
        return Err(CollisionError::InvalidContact);
    }
    if body_a < body_b {
        let (left, right) = bodies.split_at_mut(body_b);
        Ok((
            left.get_mut(body_a).ok_or(CollisionError::InvalidContact)?,
            right.first_mut().ok_or(CollisionError::InvalidContact)?,
        ))
    } else {
        let (left, right) = bodies.split_at_mut(body_a);
        Ok((
            right.first_mut().ok_or(CollisionError::InvalidContact)?,
            left.get_mut(body_b).ok_or(CollisionError::InvalidContact)?,
        ))
    }
}

fn contact_at(
    manifolds: &[SolverManifold],
    point: SolverPoint,
) -> Result<&ContactPoint, CollisionError> {
    manifolds
        .get(point.manifold)
        .and_then(|manifold| manifold.manifold.point(point.point))
        .ok_or(CollisionError::InvalidContact)
}

fn contact_mut(
    manifolds: &mut [SolverManifold],
    point: SolverPoint,
) -> Result<&mut ContactPoint, CollisionError> {
    manifolds
        .get_mut(point.manifold)
        .and_then(|manifold| manifold.manifold.point_mut(point.point))
        .ok_or(CollisionError::InvalidContact)
}

pub(crate) fn tangent_basis(normal: Vec3) -> Result<(Vec3, Vec3), CollisionError> {
    let axis = if normal.x.abs() <= normal.y.abs() && normal.x.abs() <= normal.z.abs() {
        Vec3::X
    } else if normal.y.abs() <= normal.z.abs() {
        Vec3::Y
    } else {
        Vec3::Z
    };
    let tangent_a = normal
        .checked_cross(axis)
        .and_then(Vec3::try_normalize)
        .ok_or(CollisionError::InvalidContact)?;
    let tangent_b = normal
        .checked_cross(tangent_a)
        .and_then(Vec3::try_normalize)
        .ok_or(CollisionError::InvalidContact)?;
    Ok((tangent_a, tangent_b))
}

fn validate_material(material: SolverMaterial) -> Result<(), CollisionError> {
    if !material.restitution.is_finite()
        || !(0.0..=1.0).contains(&material.restitution)
        || !material.static_friction.is_finite()
        || material.static_friction < 0.0
        || !material.dynamic_friction.is_finite()
        || material.dynamic_friction < 0.0
    {
        return Err(CollisionError::InvalidMaterial);
    }
    Ok(())
}

fn validate_contact(contact: ContactPoint) -> Result<(), CollisionError> {
    if !contact.local_a.is_finite()
        || !contact.local_b.is_finite()
        || !contact.normal.is_finite()
        || !contact.penetration.is_finite()
        || contact.penetration < 0.0
        || !contact.normal_impulse.is_finite()
        || contact.normal_impulse < 0.0
        || contact
            .tangent_impulse
            .iter()
            .any(|impulse| !impulse.is_finite())
    {
        return Err(CollisionError::InvalidContact);
    }
    Ok(())
}

fn map_constraint_error(error: crate::constraint::ConstraintError) -> CollisionError {
    match error {
        crate::constraint::ConstraintError::InvalidMass => CollisionError::InvalidContact,
        crate::constraint::ConstraintError::ArithmeticOverflow => {
            CollisionError::ArithmeticOverflow
        }
        crate::constraint::ConstraintError::NonConvergent => CollisionError::NonConvergent,
    }
}

#[cfg(test)]
#[path = "solver_tests.rs"]
mod tests;
