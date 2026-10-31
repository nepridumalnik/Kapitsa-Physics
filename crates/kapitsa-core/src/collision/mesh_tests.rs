use crate::{
    aabb::Aabb,
    collision::mesh::{MeshQueryWorkspace, TriangleBvh, local_query_bounds},
    shape::TriangleMesh,
    transform::Transform,
};
use kapitsa_math::{Quat, Vec3};

fn grid_mesh(size: usize) -> TriangleMesh {
    let mut vertices = Vec::new();
    for y in 0..=size {
        for x in 0..=size {
            vertices.push(Vec3::new(x as f64, y as f64, 0.0));
        }
    }
    let stride = size + 1;
    let mut triangles = Vec::new();
    for y in 0..size {
        for x in 0..size {
            let first = y * stride + x;
            let second = first + 1;
            let third = first + stride;
            let fourth = third + 1;
            triangles.push([first, second, fourth]);
            triangles.push([first, fourth, third]);
        }
    }
    TriangleMesh::new(vertices, triangles).unwrap()
}

#[test]
fn local_bvh_queries_only_intersecting_mesh_regions() {
    let mesh = grid_mesh(16);
    let bvh = TriangleBvh::build(&mesh).unwrap();
    let mut workspace = MeshQueryWorkspace::default();
    let mut output = Vec::new();
    let query = Aabb::new(Vec3::new(7.1, 7.1, -0.1), Vec3::new(7.9, 7.9, 0.1)).unwrap();
    bvh.query(query, &mut workspace, &mut output).unwrap();
    assert!(!output.is_empty());
    assert!(output.len() < bvh.triangle_count() / 4);
    assert!(output.windows(2).all(|pair| pair[0] < pair[1]));
}

#[test]
fn bvh_queries_match_brute_force_bounds_for_varied_regions() {
    let mesh = grid_mesh(8);
    let bvh = TriangleBvh::build(&mesh).unwrap();
    let mut workspace = MeshQueryWorkspace::default();
    let mut output = Vec::new();
    let triangles = mesh.triangles();
    let vertices = mesh.vertices();
    for x in 0..8 {
        for y in 0..8 {
            let query = Aabb::new(
                Vec3::new(x as f64 + 0.25, y as f64 + 0.25, -0.01),
                Vec3::new(x as f64 + 0.75, y as f64 + 0.75, 0.01),
            )
            .unwrap();
            let mut expected = triangles
                .iter()
                .enumerate()
                .filter_map(|(index, triangle)| {
                    let [a, b, c] = *triangle;
                    let points = [vertices[a], vertices[b], vertices[c]];
                    let min = points[0].min(points[1]).min(points[2]);
                    let max = points[0].max(points[1]).max(points[2]);
                    Aabb::new(min, max)
                        .ok()
                        .filter(|bounds| bounds.intersects(&query))
                        .map(|_| index)
                })
                .collect::<Vec<_>>();
            bvh.query(query, &mut workspace, &mut output).unwrap();
            expected.sort_unstable();
            assert_eq!(output, expected);
        }
    }
}

#[test]
fn world_query_bounds_are_conservatively_transformed_to_mesh_local_space() {
    let rotation = Quat::from_axis_angle(Vec3::Z, std::f64::consts::FRAC_PI_2).unwrap();
    let transform = Transform::new(Vec3::new(3.0, -2.0, 1.0), rotation).unwrap();
    let local = Aabb::new(Vec3::new(-0.25, -0.5, -0.1), Vec3::new(0.25, 0.5, 0.1)).unwrap();
    let world = local.transformed(&transform).unwrap();
    let recovered = local_query_bounds(world, &transform).unwrap();
    assert!(recovered.min().x <= local.min().x);
    assert!(recovered.min().y <= local.min().y);
    assert!(recovered.min().z <= local.min().z);
    assert!(recovered.max().x >= local.max().x);
    assert!(recovered.max().y >= local.max().y);
    assert!(recovered.max().z >= local.max().z);
}

#[test]
fn bvh_query_includes_triangles_touching_the_query_bounds() {
    let mesh = grid_mesh(1);
    let bvh = TriangleBvh::build(&mesh).unwrap();
    let mut workspace = MeshQueryWorkspace::default();
    let mut output = Vec::new();
    let query = Aabb::new(Vec3::ZERO, Vec3::ZERO).unwrap();
    bvh.query(query, &mut workspace, &mut output).unwrap();
    assert_eq!(output, vec![0, 1]);
}

#[test]
fn refitting_bvh_moves_triangle_bounds_and_rejects_invalid_pose_without_mutation() {
    let mesh = grid_mesh(1);
    let mut bvh = TriangleBvh::build(&mesh).unwrap();
    let moved = mesh
        .vertices()
        .iter()
        .map(|vertex| *vertex + Vec3::new(10.0, 0.0, 0.0))
        .collect::<Vec<_>>();
    bvh.refit_vertices(&mesh, &moved).unwrap();

    let mut workspace = MeshQueryWorkspace::default();
    let mut output = Vec::new();
    bvh.query(
        Aabb::new(Vec3::new(10.0, 0.0, 0.0), Vec3::new(11.0, 1.0, 0.0)).unwrap(),
        &mut workspace,
        &mut output,
    )
    .unwrap();
    assert_eq!(output, vec![0, 1]);

    assert_eq!(
        bvh.refit_vertices(&mesh, &[]),
        Err(crate::collision::CollisionError::InvalidSkinPose)
    );
    bvh.query(
        Aabb::new(Vec3::new(10.0, 0.0, 0.0), Vec3::new(11.0, 1.0, 0.0)).unwrap(),
        &mut workspace,
        &mut output,
    )
    .unwrap();
    assert_eq!(output, vec![0, 1]);
}

#[test]
fn prepared_refit_leaves_committed_bvh_unchanged_until_commit() {
    let mesh = grid_mesh(1);
    let bvh = TriangleBvh::build(&mesh).unwrap();
    let moved = mesh
        .vertices()
        .iter()
        .map(|vertex| *vertex + Vec3::new(10.0, 0.0, 0.0))
        .collect::<Vec<_>>();
    let mut scratch = super::BvhRefitScratch::default();
    bvh.prepare_refit(&mesh, &moved, &mut scratch).unwrap();

    let mut workspace = MeshQueryWorkspace::default();
    let mut output = Vec::new();
    bvh.query(
        Aabb::new(Vec3::ZERO, Vec3::ONE).unwrap(),
        &mut workspace,
        &mut output,
    )
    .unwrap();
    assert_eq!(output, vec![0, 1]);
    bvh.query(
        Aabb::new(Vec3::new(10.0, 0.0, 0.0), Vec3::new(11.0, 1.0, 0.0)).unwrap(),
        &mut workspace,
        &mut output,
    )
    .unwrap();
    assert!(output.is_empty());

    let mut bvh = bvh;
    bvh.commit_refit(&mut scratch);
    bvh.query(
        Aabb::new(Vec3::new(10.0, 0.0, 0.0), Vec3::new(11.0, 1.0, 0.0)).unwrap(),
        &mut workspace,
        &mut output,
    )
    .unwrap();
    assert_eq!(output, vec![0, 1]);
}
