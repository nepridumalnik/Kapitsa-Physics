use kapitsa_core::{
    collider::Collider,
    gravity::CentralGravity,
    joint::{FixedJoint, Joint, JointCommon, JointFrame},
    mass::MassProperties,
    rigid_body::RigidBody,
    shape::{Cuboid, Plane, Shape},
    transform::Transform,
    world::{BodyId, ForceSourceId, JointId, PhysicsWorld},
};
use kapitsa_math::{Mat3, Quat, Vec3};

pub struct ModularRocket {
    pub world: PhysicsWorld,
    pub parts: Vec<BodyId>,
    pub lower_parts: Vec<BodyId>,
    pub upper_parts: Vec<BodyId>,
    pub lower_root: BodyId,
    pub upper_root: BodyId,
    pub separation_joint: JointId,
    pub lower_engine: ForceSourceId,
    pub upper_engine: ForceSourceId,
    pub lower_tanks: Vec<BodyId>,
    pub upper_tanks: Vec<BodyId>,
}

impl ModularRocket {
    pub fn new(total_parts: usize) -> Self {
        assert!(total_parts >= 8 && total_parts.is_multiple_of(2));
        let stage_parts = total_parts / 2;
        let mut world = PhysicsWorld::new();
        let gravity_radius = 6_371_000.0;
        world.set_central_gravity(Some(
            CentralGravity::new(
                Vec3::new(0.0, -gravity_radius, 0.0),
                9.81 * gravity_radius * gravity_radius,
            )
            .expect("surface gravity"),
        ));

        let surface = world
            .add_body(RigidBody::new_static(Transform::identity()))
            .expect("surface body");
        world
            .add_collider(
                surface,
                Collider::new(
                    Shape::Plane(Plane::new(0.0, 1.0, 0.0, 0.0).expect("surface shape")),
                    Transform::identity(),
                ),
            )
            .expect("surface collider");

        let mut stages: [Vec<BodyId>; 2] = std::array::from_fn(|_| Vec::with_capacity(stage_parts));
        let mut tanks: [Vec<BodyId>; 2] = std::array::from_fn(|_| Vec::new());
        for stage in 0..2 {
            let root_position = Vec3::new(0.0, if stage == 0 { 0.5 } else { 1.5 }, 0.0);
            let trunk_parts = ((stage_parts as f64).sqrt() as usize).max(2);
            let mut positions = Vec::with_capacity(stage_parts);
            for index in 0..stage_parts {
                let (position, parent) = if index == 0 {
                    (root_position, None)
                } else if index < trunk_parts {
                    let offset = Vec3::new(0.0, 0.22, 0.0);
                    (positions[index - 1] + offset, Some((index - 1, offset)))
                } else {
                    let branch = index - trunk_parts;
                    let parent = (branch / 2) % (trunk_parts - 1);
                    let layer = branch / (2 * (trunk_parts - 1));
                    let side = if branch.is_multiple_of(2) { 1.0 } else { -1.0 };
                    let offset = Vec3::new(
                        side * (0.16 + layer as f64 * 0.01),
                        0.12 + layer as f64 * 0.01,
                        if branch.is_multiple_of(3) {
                            0.08
                        } else {
                            -0.08
                        },
                    );
                    (positions[parent] + offset, Some((parent, offset)))
                };

                let root = index == 0;
                let mass = if root {
                    5.0
                } else if index.is_multiple_of(5) {
                    1.2 + (index % 3) as f64 * 0.2
                } else {
                    0.45 + (index % 7) as f64 * 0.09
                };
                let body = RigidBody::new_dynamic(
                    Transform::new(position, Quat::IDENTITY).expect("part transform"),
                    rocket_part_mass(root, mass),
                )
                .expect("rocket part");
                let body = world.add_body(body).expect("rocket part body");
                stages[stage].push(body);
                positions.push(position);
                if let Some((parent, offset)) = parent {
                    world
                        .add_joint(stages[stage][parent], body, fixed_joint_for_offset(offset))
                        .expect("rocket module joint");
                }
                if root || index.is_multiple_of(5) {
                    tanks[stage].push(body);
                }
                if root {
                    world
                        .add_collider(
                            body,
                            Collider::new(
                                Shape::Cuboid(
                                    Cuboid::new(0.5, 0.5, 0.5).expect("rocket root shape"),
                                ),
                                Transform::identity(),
                            ),
                        )
                        .expect("rocket root collider");
                }
            }
        }

        let lower_root = stages[0][0];
        let upper_root = stages[1][0];
        let separation_joint = world
            .add_joint(lower_root, upper_root, fixed_joint_for_offset(Vec3::Y))
            .expect("stage connection");
        let lower_engine = world
            .add_force_source(lower_root, Vec3::new(-0.04, -0.2, 0.0), Vec3::Y * 800.0)
            .expect("booster engine");
        let upper_engine = world
            .add_force_source(upper_root, Vec3::new(-0.04, -0.2, 0.0), Vec3::Y * 800.0)
            .expect("upper stage engine");
        world
            .set_force_source_enabled(upper_engine, false)
            .expect("upper engine cutoff");

        let mut parts = stages[0].clone();
        parts.extend_from_slice(&stages[1]);
        Self {
            world,
            parts,
            lower_parts: stages[0].clone(),
            upper_parts: stages[1].clone(),
            lower_root,
            upper_root,
            separation_joint,
            lower_engine,
            upper_engine,
            lower_tanks: tanks[0].clone(),
            upper_tanks: tanks[1].clone(),
        }
    }
}

pub fn rocket_part_mass(root: bool, mass: f64) -> MassProperties {
    let center = if root {
        Vec3::new(-0.05, 0.0, 0.0)
    } else {
        Vec3::ZERO
    };
    let scale = if root {
        2.0
    } else {
        mass * (0.035 + (mass % 3.0) * 0.002)
    };
    MassProperties::new(
        mass,
        center,
        Mat3::from_diagonal(Vec3::new(scale, scale * 1.2, scale * 0.8)),
    )
    .expect("part mass")
}

pub fn fixed_joint_for_offset(offset: Vec3) -> Joint {
    let common = JointCommon::new(
        JointFrame::new(offset * 0.5, Quat::IDENTITY),
        JointFrame::new(offset * -0.5, Quat::IDENTITY),
    );
    Joint::Fixed(FixedJoint::new(common))
}

#[cfg(test)]
mod tests {
    use std::collections::{HashMap, VecDeque};

    use super::ModularRocket;

    #[test]
    fn requested_rocket_sizes_have_connected_repeatable_joint_graphs() {
        for total_parts in [100, 200, 500, 1000] {
            let first = ModularRocket::new(total_parts);
            let repeat = ModularRocket::new(total_parts);
            assert_eq!(first.parts.len(), total_parts);
            assert_eq!(first.world.joint_count(), total_parts - 1);
            assert_eq!(
                first.lower_parts.len() + first.upper_parts.len(),
                total_parts
            );
            assert!(!first.lower_tanks.is_empty() && !first.upper_tanks.is_empty());

            let indices = first
                .parts
                .iter()
                .enumerate()
                .map(|(index, &body)| (body, index))
                .collect::<HashMap<_, _>>();
            let mut edges = Vec::with_capacity(total_parts - 1);
            let mut adjacency = vec![Vec::new(); total_parts];
            let mut degree = vec![0; total_parts];
            for (_, body_a, body_b, _) in first.world.joints() {
                let a = indices[&body_a];
                let b = indices[&body_b];
                edges.push((a.min(b), a.max(b)));
                adjacency[a].push(b);
                adjacency[b].push(a);
                degree[a] += 1;
                degree[b] += 1;
            }
            edges.sort_unstable();
            let mut visited = vec![false; total_parts];
            let mut queue = VecDeque::from([0]);
            visited[0] = true;
            while let Some(current) = queue.pop_front() {
                for &next in &adjacency[current] {
                    if !visited[next] {
                        visited[next] = true;
                        queue.push_back(next);
                    }
                }
            }
            assert!(visited.into_iter().all(|seen| seen), "{total_parts} parts");
            assert!(edges.contains(&(0, 1)), "linear trunk");
            assert!(degree.iter().any(|&degree| degree >= 3), "branches");

            let repeated_indices = repeat
                .parts
                .iter()
                .enumerate()
                .map(|(index, &body)| (body, index))
                .collect::<HashMap<_, _>>();
            let mut repeated_edges = repeat
                .world
                .joints()
                .map(|(_, a, b, _)| {
                    let a = repeated_indices[&a];
                    let b = repeated_indices[&b];
                    (a.min(b), a.max(b))
                })
                .collect::<Vec<_>>();
            repeated_edges.sort_unstable();
            assert_eq!(edges, repeated_edges, "topology must be deterministic");

            let masses = first
                .parts
                .iter()
                .map(|&id| {
                    first
                        .world
                        .body(id)
                        .and_then(|body| body.physics().mass())
                        .expect("dynamic part has mass")
                        .mass()
                        .to_bits()
                })
                .collect::<std::collections::HashSet<_>>();
            assert!(masses.len() > 2, "parts use varied masses");
        }
    }
}
