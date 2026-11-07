use kapitsa::{
    Collider, Joint, JointTarget, MassProperties, PhysicsMaterial, RigidBody, Shape, Simulation,
    Transform,
    cluster::{
        ClusterPartDesc, MassChangeMode, MassTransfer, WeldDesc, WeldStiffness, WeldStrength,
    },
    joint::{FixedJoint, JointCommon, JointFrame},
    math::{Mat3, Vec3},
    shape::Sphere,
};

fn mass(value: f64) -> MassProperties {
    MassProperties::new(value, Vec3::ZERO, Mat3::IDENTITY).unwrap()
}

fn weld() -> WeldDesc {
    WeldDesc {
        frame_a: JointFrame::IDENTITY,
        frame_b: JointFrame::IDENTITY,
        strength: WeldStrength::Unbreakable,
        stiffness: WeldStiffness {
            linear: 1.0,
            angular: 1.0,
        },
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn materials_are_managed_through_client_on_pause() {
    let mut simulation = Simulation::builder()
        .max_workers(2)
        .spawn_on_tokio(&tokio::runtime::Handle::current())
        .unwrap();
    simulation.pause().unwrap();
    let client = simulation.client();
    let body = client
        .add_body(RigidBody::new_static(Transform::identity()))
        .await
        .unwrap();
    let static_part = client
        .enable_cluster_with_mass(body, mass(1.0))
        .await
        .unwrap();
    assert_eq!(
        client
            .cluster_part(static_part)
            .await
            .unwrap()
            .unwrap()
            .body(),
        body
    );
    let collider = client
        .add_part_collider(
            static_part,
            Collider::new(
                Shape::Sphere(Sphere::new(1.0).unwrap()),
                Transform::identity(),
            ),
        )
        .await
        .unwrap();
    let second_collider = client
        .add_part_collider(
            static_part,
            Collider::new(
                Shape::Sphere(Sphere::new(0.5).unwrap()),
                Transform::identity(),
            ),
        )
        .await
        .unwrap();
    let step = client.snapshot().step();
    let initial = PhysicsMaterial::new(0.2, 0.3, 0.1, 1.0).unwrap();
    let replacement = PhysicsMaterial::new(0.4, 0.5, 0.2, 2.0).unwrap();
    let id = client.add_material(initial).await.unwrap();
    assert_eq!(client.material(id).await.unwrap(), Some(initial));
    client
        .set_collider_material(collider, Some(id))
        .await
        .unwrap();
    client
        .set_collider_material(second_collider, Some(id))
        .await
        .unwrap();
    assert!(client.remove_material(id).await.is_err());
    client.update_material(id, replacement).await.unwrap();
    assert_eq!(client.material(id).await.unwrap(), Some(replacement));
    client.set_collider_material(collider, None).await.unwrap();
    assert!(client.remove_material(id).await.is_err());
    client
        .set_collider_material(second_collider, None)
        .await
        .unwrap();
    client.remove_material(id).await.unwrap();
    assert_eq!(client.material(id).await.unwrap(), None);
    assert!(client.update_material(id, initial).await.is_err());
    assert_eq!(client.snapshot().step(), step);
    simulation.stop().await.unwrap();
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn clients_reject_foreign_and_stale_material_ids() {
    let mut first = Simulation::builder()
        .max_workers(1)
        .spawn_on_tokio(&tokio::runtime::Handle::current())
        .unwrap();
    let mut second = Simulation::builder()
        .max_workers(1)
        .spawn_on_tokio(&tokio::runtime::Handle::current())
        .unwrap();
    first.pause().unwrap();
    second.pause().unwrap();
    let a = first.client();
    let b = second.client();
    let material = PhysicsMaterial::new(0.2, 0.3, 0.1, 1.0).unwrap();
    let id = a.add_material(material).await.unwrap();
    assert_eq!(b.material(id).await.unwrap(), None);
    assert!(b.update_material(id, material).await.is_err());
    a.remove_material(id).await.unwrap();
    let replacement = a.add_material(material).await.unwrap();
    assert_ne!(id, replacement);
    assert!(a.update_material(id, material).await.is_err());
    first.stop().await.unwrap();
    second.stop().await.unwrap();
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn clusters_can_split_merge_and_inspect_ids_on_pause() {
    let mut simulation = Simulation::builder()
        .max_workers(2)
        .spawn_on_tokio(&tokio::runtime::Handle::current())
        .unwrap();
    simulation.pause().unwrap();
    let client = simulation.client();
    let body = client
        .add_body(RigidBody::new_dynamic(Transform::identity(), mass(1.0)).unwrap())
        .await
        .unwrap();
    let root = client.enable_cluster(body).await.unwrap();
    let part = client
        .add_cluster_part(
            root,
            ClusterPartDesc {
                local_transform: Transform::identity(),
                mass: mass(1.0),
                colliders: Vec::new(),
                linear_velocity: Vec3::ZERO,
                angular_velocity: Vec3::ZERO,
            },
            weld(),
        )
        .await
        .unwrap();
    let temporary = client
        .add_cluster_part(
            root,
            ClusterPartDesc {
                local_transform: Transform::identity(),
                mass: mass(1.0),
                colliders: Vec::new(),
                linear_velocity: Vec3::ZERO,
                angular_velocity: Vec3::ZERO,
            },
            weld(),
        )
        .await
        .unwrap();
    client.destroy_cluster_part(temporary).await.unwrap();
    assert!(client.cluster_part(temporary).await.unwrap().is_none());
    let cluster = client.cluster(body).await.unwrap().unwrap();
    assert_eq!(cluster.part_count(), 2);
    let mut invalid_weld = weld();
    invalid_weld.stiffness.linear = 0.0;
    assert!(
        client
            .add_cluster_part(
                root,
                ClusterPartDesc {
                    local_transform: Transform::identity(),
                    mass: mass(1.0),
                    colliders: Vec::new(),
                    linear_velocity: Vec3::ZERO,
                    angular_velocity: Vec3::ZERO
                },
                invalid_weld
            )
            .await
            .is_err()
    );
    assert_eq!(client.cluster(body).await.unwrap().unwrap(), cluster);
    let weld_id = cluster.welds()[0];
    assert_eq!(
        client.weld(weld_id).await.unwrap().unwrap().parts(),
        (root, part)
    );
    assert_eq!(
        client.cluster_part(part).await.unwrap().unwrap().body(),
        body
    );
    let collider = client
        .add_part_collider(
            part,
            Collider::new(
                Shape::Sphere(Sphere::new(0.2).unwrap()),
                Transform::identity(),
            ),
        )
        .await
        .unwrap();
    assert!(
        client
            .cluster_part(part)
            .await
            .unwrap()
            .unwrap()
            .colliders()
            .contains(&collider)
    );
    client.remove_part_collider(collider).await.unwrap();
    client
        .set_part_mass(part, mass(2.0), MassChangeMode::KeepVelocityField)
        .await
        .unwrap();
    client
        .set_part_mass_with_transfer(
            part,
            mass(2.0),
            MassTransfer {
                world_origin: Vec3::ZERO,
                linear_impulse: Vec3::ZERO,
                angular_impulse: Vec3::ZERO,
            },
        )
        .await
        .unwrap();
    let external = client
        .add_body(RigidBody::new_static(Transform::identity()))
        .await
        .unwrap();
    let joint = Joint::Fixed(FixedJoint::new(JointCommon::new(
        JointFrame::IDENTITY,
        JointFrame::IDENTITY,
    )));
    let joint_id = client
        .add_joint_targets(JointTarget::Part(part), JointTarget::Body(external), joint)
        .await
        .unwrap();
    let force = client
        .add_part_force_source(part, Vec3::ZERO, Vec3::Y)
        .await
        .unwrap();
    let bodies = client.detach_part(part).await.unwrap();
    assert_eq!(bodies.len(), 2);
    assert!(client.weld(weld_id).await.unwrap().is_none());
    let part_body = client.cluster_part(part).await.unwrap().unwrap().body();
    assert_ne!(part_body, body);
    assert!(client.snapshot().body(part_body).is_some());
    client.update_joint_targets(joint_id, joint).await.unwrap();
    client.remove_joint(joint_id).await.unwrap();
    client.remove_force_source(force).await.unwrap();
    client
        .merge_clusters(body, part_body, weld())
        .await
        .unwrap();
    let merged = client.cluster(body).await.unwrap().unwrap();
    assert_eq!(merged.part_count(), 2);
    let bodies = client.break_weld(merged.welds()[0]).await.unwrap();
    assert_eq!(bodies.len(), 2);
    assert!(client.break_weld(weld_id).await.is_err());
    let part_body = client.cluster_part(part).await.unwrap().unwrap().body();
    assert!(client.destroy_cluster_part(part).await.is_err());
    assert!(client.cluster_part(part).await.unwrap().is_some());
    assert!(client.snapshot().body(part_body).is_some());
    simulation.stop().await.unwrap();
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn detached_part_keeps_its_force_source_after_resuming() {
    let mut simulation = Simulation::builder()
        .frequency(120)
        .max_workers(2)
        .spawn_on_tokio(&tokio::runtime::Handle::current())
        .unwrap();
    simulation.pause().unwrap();
    let mut client = simulation.client();
    let body = client
        .add_body(RigidBody::new_dynamic(Transform::identity(), mass(1.0)).unwrap())
        .await
        .unwrap();
    let root = client.enable_cluster(body).await.unwrap();
    let part = client
        .add_cluster_part(
            root,
            ClusterPartDesc {
                local_transform: Transform::identity(),
                mass: mass(1.0),
                colliders: Vec::new(),
                linear_velocity: Vec3::ZERO,
                angular_velocity: Vec3::ZERO,
            },
            weld(),
        )
        .await
        .unwrap();
    client
        .add_part_force_source(part, Vec3::ZERO, Vec3::Y)
        .await
        .unwrap();
    client.detach_part(part).await.unwrap();
    let detached = client.cluster_part(part).await.unwrap().unwrap().body();
    simulation.resume().unwrap();
    tokio::time::timeout(std::time::Duration::from_secs(2), async {
        loop {
            let snapshot = client.next_snapshot().await.unwrap();
            if snapshot.body(detached).unwrap().linear_velocity().y > 0.0 {
                assert_eq!(snapshot.body(body).unwrap().linear_velocity(), Vec3::ZERO);
                break;
            }
        }
    })
    .await
    .unwrap();
    simulation.stop().await.unwrap();
}
