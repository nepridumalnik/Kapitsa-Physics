use crate::{
    aabb::Aabb,
    collision::height_field::{query_triangles, query_world_bounds},
    shape::HeightField,
};
use kapitsa_math::{Vec2, Vec3};

#[test]
fn height_field_cells_use_a_fixed_diagonal_and_stable_features() {
    let field = HeightField::from_vec(2, 2, Vec2::new(2.0, 3.0), vec![0.0, 0.0, 0.0, 1.0]).unwrap();
    let mut triangles = Vec::new();
    query_triangles(
        &field,
        Aabb::new(Vec3::ZERO, Vec3::new(2.0, 3.0, 1.0)).unwrap(),
        &mut triangles,
    )
    .unwrap();
    assert_eq!(triangles.len(), 2);
    assert_eq!(triangles[0].vertices[0], Vec3::ZERO);
    assert_eq!(triangles[0].vertices[1], Vec3::new(2.0, 0.0, 0.0));
    assert_eq!(triangles[0].vertices[2], Vec3::new(2.0, 3.0, 1.0));
    assert_eq!(triangles[1].vertices[1], Vec3::new(2.0, 3.0, 1.0));
    assert_eq!(triangles[0].feature_id, 0);
    assert_eq!(triangles[1].feature_id, 1);
}

#[test]
fn height_field_query_visits_only_cells_in_the_local_region() {
    let field = HeightField::from_vec(12, 12, Vec2::ONE, vec![0.0; 144]).unwrap();
    let mut triangles = Vec::new();
    query_triangles(
        &field,
        Aabb::new(Vec3::new(5.1, 6.1, -0.1), Vec3::new(5.9, 6.9, 0.1)).unwrap(),
        &mut triangles,
    )
    .unwrap();
    assert_eq!(triangles.len(), 2);
    assert_eq!(triangles[0].feature_id, 142);
    assert_eq!(triangles[1].feature_id, 143);
}

#[test]
fn height_field_query_checks_triangle_bounds_and_grid_edges() {
    let field = HeightField::from_vec(3, 3, Vec2::ONE, vec![0.0; 9]).unwrap();
    let mut triangles = Vec::new();
    query_triangles(
        &field,
        Aabb::new(Vec3::new(1.0, 0.0, 2.0), Vec3::new(2.0, 1.0, 3.0)).unwrap(),
        &mut triangles,
    )
    .unwrap();
    assert!(triangles.is_empty());

    query_triangles(
        &field,
        Aabb::new(Vec3::new(2.0, 2.0, 0.0), Vec3::new(2.0, 2.0, 0.0)).unwrap(),
        &mut triangles,
    )
    .unwrap();
    assert_eq!(triangles.len(), 2);
    assert!(triangles.iter().all(|triangle| triangle.feature_id >= 6));
}

#[test]
fn height_field_world_queries_respect_collider_transforms() {
    let field = HeightField::from_vec(2, 2, Vec2::ONE, vec![0.0; 4]).unwrap();
    let rotation =
        kapitsa_math::Quat::from_axis_angle(Vec3::Z, std::f64::consts::FRAC_PI_2).unwrap();
    let transform = crate::transform::Transform::new(Vec3::new(5.0, -2.0, 1.0), rotation).unwrap();
    let world_bounds = Aabb::new(Vec3::new(3.9, -1.1, 0.9), Vec3::new(4.1, -0.9, 1.1)).unwrap();
    let mut triangles = Vec::new();
    query_world_bounds(&field, &transform, world_bounds, &mut triangles).unwrap();
    assert_eq!(triangles.len(), 2);
}
