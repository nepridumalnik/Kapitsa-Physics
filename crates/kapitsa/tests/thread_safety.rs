use kapitsa::prelude::*;

fn assert_send_sync<T: Send + Sync>() {}

#[test]
fn core_public_types_are_send_and_sync() {
    assert_send_sync::<Error>();
    assert_send_sync::<PhysicsWorld>();
    assert_send_sync::<RigidBody>();
    assert_send_sync::<Transform>();
    assert_send_sync::<MassProperties>();
    assert_send_sync::<Shape>();
    assert_send_sync::<PhysicsMaterial>();
}

#[cfg(feature = "service")]
#[test]
fn service_public_types_are_send_and_sync() {
    assert_send_sync::<Simulation>();
    assert_send_sync::<PhysicsClient>();
    assert_send_sync::<PhysicsSnapshot>();
    assert_send_sync::<SimulationConfig>();
}
