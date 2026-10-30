use kapitsa_core::{
    collider::{Collider, CollisionFilter},
    mass::MassProperties,
    material::MaterialId,
    rigid_body::RigidBody,
    transform::Transform,
    world::{BodyId, ColliderId, PhysicsWorld, WorldError},
};
use kapitsa_math::Vec3;
use tokio::sync::oneshot;

use crate::PhysicsClientError;

/// An operation applied to the simulation world at a physics-step boundary.
pub enum PhysicsCommand {
    /// Inserts a rigid body and returns its new [`BodyId`].
    AddBody { body: RigidBody },
    /// Removes a body and its attached colliders.
    RemoveBody { id: BodyId },
    /// Attaches a collider to a body and returns its new [`ColliderId`].
    AddCollider { body: BodyId, collider: Collider },
    /// Removes a collider.
    RemoveCollider { id: ColliderId },
    /// Replaces a body's transform while preserving its velocities.
    SetTransform { id: BodyId, transform: Transform },
    /// Sets the world-space linear velocity.
    SetLinearVelocity { id: BodyId, velocity: Vec3 },
    /// Sets the world-space angular velocity.
    SetAngularVelocity { id: BodyId, velocity: Vec3 },
    /// Replaces the body's dynamic mass properties.
    SetMass { id: BodyId, mass: MassProperties },
    /// Sleeps a dynamic body and clears its motion and forces.
    Sleep { id: BodyId },
    /// Wakes a dynamic body.
    WakeUp { id: BodyId },
    /// Accumulates a world-space force.
    ApplyForce { id: BodyId, force: Vec3 },
    /// Accumulates a world-space force at a world-space point.
    ApplyForceAtPoint {
        id: BodyId,
        force: Vec3,
        point: Vec3,
    },
    /// Accumulates a world-space torque.
    ApplyTorque { id: BodyId, torque: Vec3 },
    /// Applies a world-space impulse immediately.
    ApplyImpulse { id: BodyId, impulse: Vec3 },
    /// Applies a world-space impulse at a world-space point immediately.
    ApplyImpulseAtPoint {
        id: BodyId,
        impulse: Vec3,
        point: Vec3,
    },
    /// Clears accumulated force and torque.
    ClearForces { id: BodyId },
    /// Replaces a collider's collision filter.
    SetColliderFilter {
        id: ColliderId,
        filter: CollisionFilter,
    },
    /// Assigns a material ID without resolving it in a material store.
    SetColliderMaterial {
        id: ColliderId,
        material: Option<MaterialId>,
    },
}

/// Successful result returned by a physics command.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PhysicsCommandResult {
    /// The command completed without returning a newly allocated ID.
    Done,
    /// A body was inserted.
    Body(BodyId),
    /// A collider was inserted.
    Collider(ColliderId),
}

pub(crate) struct CommandRequest {
    pub command: PhysicsCommand,
    pub reply: oneshot::Sender<Result<PhysicsCommandResult, PhysicsClientError>>,
}

pub(crate) fn apply_command(
    world: &mut PhysicsWorld,
    command: PhysicsCommand,
) -> Result<PhysicsCommandResult, PhysicsClientError> {
    let result = match command {
        PhysicsCommand::AddBody { body } => PhysicsCommandResult::Body(world.add_body(body)?),
        PhysicsCommand::RemoveBody { id } => {
            world.remove_body(id).ok_or(WorldError::InvalidBodyId)?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::AddCollider { body, collider } => {
            PhysicsCommandResult::Collider(world.add_collider(body, collider)?)
        }
        PhysicsCommand::RemoveCollider { id } => {
            world
                .remove_collider(id)
                .ok_or(WorldError::InvalidColliderId)?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::SetTransform { id, transform } => {
            world
                .body_mut(id)
                .ok_or(WorldError::InvalidBodyId)?
                .set_transform(transform)?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::SetLinearVelocity { id, velocity } => {
            world
                .body_mut(id)
                .ok_or(WorldError::InvalidBodyId)?
                .set_linear_velocity(velocity)?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::SetAngularVelocity { id, velocity } => {
            world
                .body_mut(id)
                .ok_or(WorldError::InvalidBodyId)?
                .set_angular_velocity(velocity)?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::SetMass { id, mass } => {
            world
                .body_mut(id)
                .ok_or(WorldError::InvalidBodyId)?
                .set_mass(mass)?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::Sleep { id } => {
            world
                .body_mut(id)
                .ok_or(WorldError::InvalidBodyId)?
                .sleep()?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::WakeUp { id } => {
            world
                .body_mut(id)
                .ok_or(WorldError::InvalidBodyId)?
                .wake_up();
            PhysicsCommandResult::Done
        }
        PhysicsCommand::ApplyForce { id, force } => {
            world
                .body_mut(id)
                .ok_or(WorldError::InvalidBodyId)?
                .apply_force(force)?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::ApplyForceAtPoint { id, force, point } => {
            world
                .body_mut(id)
                .ok_or(WorldError::InvalidBodyId)?
                .apply_force_at_point(force, point)?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::ApplyTorque { id, torque } => {
            world
                .body_mut(id)
                .ok_or(WorldError::InvalidBodyId)?
                .apply_torque(torque)?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::ApplyImpulse { id, impulse } => {
            world
                .body_mut(id)
                .ok_or(WorldError::InvalidBodyId)?
                .apply_impulse(impulse)?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::ApplyImpulseAtPoint { id, impulse, point } => {
            world
                .body_mut(id)
                .ok_or(WorldError::InvalidBodyId)?
                .apply_impulse_at_point(impulse, point)?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::ClearForces { id } => {
            world
                .body_mut(id)
                .ok_or(WorldError::InvalidBodyId)?
                .clear_forces();
            PhysicsCommandResult::Done
        }
        PhysicsCommand::SetColliderFilter { id, filter } => {
            world
                .collider_mut(id)
                .ok_or(WorldError::InvalidColliderId)?
                .set_filter(filter);
            PhysicsCommandResult::Done
        }
        PhysicsCommand::SetColliderMaterial { id, material } => {
            world
                .collider_mut(id)
                .ok_or(WorldError::InvalidColliderId)?
                .set_material(material);
            PhysicsCommandResult::Done
        }
    };
    Ok(result)
}

#[cfg(test)]
mod tests {
    use kapitsa_core::{
        collider::{Collider, CollisionFilter},
        mass::MassProperties,
        rigid_body::{BodyError, RigidBody},
        shape::{Shape, Sphere},
        transform::Transform,
        world::{PhysicsWorld, WorldError},
    };
    use kapitsa_math::{Mat3, Quat, Vec3};

    use super::{PhysicsCommand, PhysicsCommandResult, apply_command};
    use crate::PhysicsClientError;

    fn mass(value: f64) -> MassProperties {
        MassProperties::new(value, Vec3::ZERO, Mat3::IDENTITY).expect("valid mass")
    }

    fn dynamic_body() -> RigidBody {
        RigidBody::new_dynamic(Transform::identity(), mass(1.0)).expect("dynamic body")
    }

    fn collider() -> Collider {
        Collider::new(
            Shape::Sphere(Sphere::new(1.0).expect("valid sphere")),
            Transform::identity(),
        )
    }

    fn assert_done(result: Result<PhysicsCommandResult, PhysicsClientError>) {
        assert!(matches!(result, Ok(PhysicsCommandResult::Done)));
    }

    #[test]
    fn add_remove_and_reused_body_ids_are_handled_by_world() {
        let mut world = PhysicsWorld::new();
        let PhysicsCommandResult::Body(first) = apply_command(
            &mut world,
            PhysicsCommand::AddBody {
                body: dynamic_body(),
            },
        )
        .expect("add body") else {
            panic!("add body must return its ID");
        };
        assert!(world.body(first).is_some());
        assert_done(apply_command(
            &mut world,
            PhysicsCommand::RemoveBody { id: first },
        ));
        assert!(world.body(first).is_none());

        let PhysicsCommandResult::Body(replacement) = apply_command(
            &mut world,
            PhysicsCommand::AddBody {
                body: dynamic_body(),
            },
        )
        .expect("reuse body slot") else {
            panic!("add body must return its ID");
        };
        assert_eq!(first.index(), replacement.index());
        assert_ne!(first.generation(), replacement.generation());
        assert!(matches!(
            apply_command(
                &mut world,
                PhysicsCommand::SetTransform {
                    id: first,
                    transform: Transform::identity(),
                }
            ),
            Err(PhysicsClientError::World(WorldError::InvalidBodyId))
        ));
    }

    #[test]
    fn commands_reject_body_ids_from_another_world() {
        let mut source = PhysicsWorld::new();
        let foreign = source
            .add_body(dynamic_body())
            .expect("source body insertion");
        let mut target = PhysicsWorld::new();
        let local = target
            .add_body(dynamic_body())
            .expect("target body insertion");

        assert!(matches!(
            apply_command(
                &mut target,
                PhysicsCommand::SetLinearVelocity {
                    id: foreign,
                    velocity: Vec3::X,
                }
            ),
            Err(PhysicsClientError::World(WorldError::InvalidBodyId))
        ));
        assert!(matches!(
            apply_command(&mut target, PhysicsCommand::RemoveBody { id: foreign }),
            Err(PhysicsClientError::World(WorldError::InvalidBodyId))
        ));
        assert_eq!(target.body_count(), 1);
        assert!(target.body(local).is_some());
        assert_eq!(
            target
                .body(local)
                .expect("local body remains")
                .state()
                .linear_velocity(),
            Vec3::ZERO
        );
    }

    #[test]
    fn body_commands_delegate_to_existing_body_operations() {
        let mut world = PhysicsWorld::new();
        let id = world.add_body(dynamic_body()).expect("body");
        let transform = Transform::new(Vec3::X, Quat::IDENTITY).expect("transform");
        let commands = [
            PhysicsCommand::SetTransform { id, transform },
            PhysicsCommand::SetLinearVelocity {
                id,
                velocity: Vec3::X,
            },
            PhysicsCommand::SetAngularVelocity {
                id,
                velocity: Vec3::Y,
            },
            PhysicsCommand::SetMass {
                id,
                mass: mass(2.0),
            },
            PhysicsCommand::ApplyForce { id, force: Vec3::X },
            PhysicsCommand::ApplyForceAtPoint {
                id,
                force: Vec3::Y,
                point: Vec3::Z,
            },
            PhysicsCommand::ApplyTorque {
                id,
                torque: Vec3::Z,
            },
            PhysicsCommand::ApplyImpulse {
                id,
                impulse: Vec3::X,
            },
            PhysicsCommand::ApplyImpulseAtPoint {
                id,
                impulse: Vec3::Y,
                point: Vec3::Z,
            },
        ];
        for command in commands {
            assert_done(apply_command(&mut world, command));
        }
        let body = world.body(id).expect("body remains");
        assert_eq!(body.state().transform(), transform);
        assert_eq!(body.physics().mass(), Some(mass(2.0)));
        assert_ne!(body.forces().force(), Vec3::ZERO);
        assert_ne!(body.forces().torque(), Vec3::ZERO);
        assert_eq!(body.state().linear_velocity(), Vec3::new(1.5, 0.5, 0.0));

        assert_done(apply_command(&mut world, PhysicsCommand::Sleep { id }));
        assert!(world.body(id).expect("body").state().is_sleeping());
        assert_done(apply_command(&mut world, PhysicsCommand::WakeUp { id }));
        assert!(!world.body(id).expect("body").state().is_sleeping());

        apply_command(
            &mut world,
            PhysicsCommand::ApplyForce { id, force: Vec3::X },
        )
        .expect("apply force");
        apply_command(&mut world, PhysicsCommand::ClearForces { id }).expect("clear forces");
        assert_eq!(world.body(id).expect("body").forces().force(), Vec3::ZERO);
        assert_eq!(world.body(id).expect("body").forces().torque(), Vec3::ZERO);
    }

    #[test]
    fn invalid_body_and_numeric_commands_return_errors_without_partial_changes() {
        let mut world = PhysicsWorld::new();
        let id = world.add_body(dynamic_body()).expect("body");
        assert!(matches!(
            apply_command(
                &mut world,
                PhysicsCommand::ApplyForce {
                    id,
                    force: Vec3::new(f64::INFINITY, 0.0, 0.0),
                }
            ),
            Err(PhysicsClientError::Body(BodyError::NonFiniteValue))
        ));
        let body = world.body(id).expect("body unchanged");
        assert_eq!(body.forces().force(), Vec3::ZERO);

        let invalid = world
            .add_body(RigidBody::new_kinematic(Transform::identity()))
            .unwrap();
        assert!(matches!(
            apply_command(&mut world, PhysicsCommand::Sleep { id: invalid }),
            Err(PhysicsClientError::Body(BodyError::BodyMustBeDynamic))
        ));
    }

    #[test]
    fn collider_commands_update_remove_and_follow_body_removal() {
        let mut world = PhysicsWorld::new();
        let body = world.add_body(dynamic_body()).expect("body");
        let PhysicsCommandResult::Collider(collider_id) = apply_command(
            &mut world,
            PhysicsCommand::AddCollider {
                body,
                collider: collider(),
            },
        )
        .expect("add collider") else {
            panic!("add collider must return its ID");
        };
        let filter = CollisionFilter::new(3, 7);
        assert_done(apply_command(
            &mut world,
            PhysicsCommand::SetColliderFilter {
                id: collider_id,
                filter,
            },
        ));
        assert_eq!(
            world.collider(collider_id).expect("collider").filter(),
            filter
        );
        apply_command(
            &mut world,
            PhysicsCommand::SetColliderMaterial {
                id: collider_id,
                material: None,
            },
        )
        .expect("clear material");
        assert_eq!(
            world.collider(collider_id).expect("collider").material(),
            None
        );
        apply_command(&mut world, PhysicsCommand::RemoveBody { id: body })
            .expect("remove body and attached collider");
        assert!(world.collider(collider_id).is_none());
        assert!(matches!(
            apply_command(
                &mut world,
                PhysicsCommand::RemoveCollider { id: collider_id }
            ),
            Err(PhysicsClientError::World(WorldError::InvalidColliderId))
        ));
    }
}
