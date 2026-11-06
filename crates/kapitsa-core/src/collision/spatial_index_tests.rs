use crate::{
    aabb::Aabb,
    collider::Collider,
    collision::spatial_index::SpatialIndex,
    shape::{Shape, Sphere},
    transform::Transform,
    world::PhysicsWorld,
};
use kapitsa_math::Vec3;

fn bounds(x: f64, y: f64, radius: f64) -> Aabb {
    Aabb::new(
        Vec3::new(x - radius, y - radius, -radius),
        Vec3::new(x + radius, y + radius, radius),
    )
    .expect("test bounds")
}

fn colliders(count: usize) -> Vec<crate::world::ColliderId> {
    let mut world = PhysicsWorld::new();
    let body = world
        .add_body(crate::rigid_body::RigidBody::new_static(
            Transform::identity(),
        ))
        .expect("body");
    (0..count)
        .map(|_| {
            world
                .add_collider(
                    body,
                    Collider::new(
                        Shape::Sphere(Sphere::new(0.5).expect("sphere")),
                        Transform::identity(),
                    ),
                )
                .expect("collider")
        })
        .collect()
}

#[test]
fn queries_match_a_linear_aabb_oracle_after_inserts_updates_and_removals() {
    let ids = colliders(200);
    let mut index = SpatialIndex::new();
    let proxies: Vec<_> = ids
        .iter()
        .enumerate()
        .map(|(index, id)| {
            let x = index as f64 * 2.0;
            let y = (index * 37 % 19) as f64 * 2.0;
            (*id, bounds(x, y, 0.75))
        })
        .collect();
    index.synchronize(&proxies).expect("initial tree sync");

    let query = bounds(120.0, 18.0, 7.0);
    let mut actual = Vec::new();
    index.query(query, &mut actual).expect("tree query");
    actual.sort();
    let mut expected: Vec<_> = proxies
        .iter()
        .filter(|(_, candidate)| candidate.intersects(&query))
        .map(|(id, _)| *id)
        .collect();
    expected.sort();
    assert_eq!(actual, expected);

    let moved = ids[0];
    let mut updated = proxies.clone();
    updated[0].1 = bounds(120.0, 18.0, 0.75);
    index.synchronize(&updated).expect("update tree");
    index.remove(ids[1]).expect("remove leaf");
    let mut actual = Vec::new();
    index.query(query, &mut actual).expect("query updated tree");
    assert!(actual.contains(&moved));
    assert!(!actual.contains(&ids[1]));
}

#[test]
fn sequential_insertions_remain_balanced_and_slots_are_reused() {
    let ids = colliders(1024);
    let mut index = SpatialIndex::new();
    for (index_value, id) in ids.iter().copied().enumerate() {
        index
            .insert(id, bounds(index_value as f64, 0.0, 0.25))
            .expect("insert leaf");
    }
    let maximum_height = (ids.len() as f64).log2().ceil() as u32 * 2 + 4;
    assert!(index.root.is_some());
    let height = index
        .node(index.root.expect("root"))
        .expect("root node")
        .height;
    assert!(
        height <= maximum_height,
        "tree height {height} > {maximum_height}"
    );

    let removed = ids[512];
    let old_node_count = index.nodes.len();
    assert!(index.remove(removed).expect("remove"));
    let replacement = colliders(1)[0];
    index
        .insert(replacement, bounds(2048.0, 0.0, 0.25))
        .expect("reuse slots");
    assert_eq!(index.nodes.len(), old_node_count);
}

#[test]
fn synchronization_rejects_duplicate_proxy_ids_without_clearing_live_tree() {
    let ids = colliders(2);
    let mut index = SpatialIndex::new();
    index.insert(ids[0], bounds(0.0, 0.0, 1.0)).expect("insert");
    let duplicate = [
        (ids[1], bounds(3.0, 0.0, 1.0)),
        (ids[1], bounds(4.0, 0.0, 1.0)),
    ];
    assert_eq!(
        index.synchronize(&duplicate),
        Err(crate::collision::CollisionError::InvalidContact)
    );
    let mut output = Vec::new();
    index
        .query(bounds(0.0, 0.0, 1.0), &mut output)
        .expect("query old tree");
    assert_eq!(output, vec![ids[0]]);
}
