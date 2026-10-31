use crate::{
    aabb::Aabb,
    collision::level_set::{cell_lipschitz_bound, query_surface_cells, query_world_cells, sample},
    shape::LevelSet,
    transform::Transform,
};
use kapitsa_math::{Quat, Vec3};

fn plane_field() -> LevelSet {
    let mut values = Vec::new();
    for _z in 0..2 {
        for _y in 0..2 {
            values.extend([-2.0, 0.0, 2.0]);
        }
    }
    LevelSet::from_vec(
        Vec3::new(-1.0, 2.0, 4.0),
        [3, 2, 2],
        Vec3::new(2.0, 3.0, 4.0),
        values,
    )
    .unwrap()
}

#[test]
fn interpolation_and_gradient_use_grid_spacing() {
    let field = plane_field();
    let sample = sample(&field, Vec3::new(0.0, 3.5, 6.0)).unwrap().unwrap();
    assert_eq!(sample.value, -1.0);
    assert_eq!(sample.gradient, Vec3::X);
    assert_eq!(sample.normal, Some(Vec3::X));
}

#[test]
fn cell_lipschitz_bound_covers_trilinear_gradient() {
    let field = plane_field();
    assert!((cell_lipschitz_bound(&field, 0).unwrap() - 1.0).abs() < 1.0e-12);
}

#[test]
fn zero_gradient_has_no_fabricated_surface_normal() {
    let field = LevelSet::from_vec(Vec3::ZERO, [2, 2, 2], Vec3::ONE, vec![0.0; 8]).unwrap();
    let sample = sample(&field, Vec3::splat(0.5)).unwrap().unwrap();
    assert_eq!(sample.value, 0.0);
    assert_eq!(sample.gradient, Vec3::ZERO);
    assert_eq!(sample.normal, None);
}

#[test]
fn interpolation_does_not_extend_beyond_the_grid() {
    let field = plane_field();
    assert!(
        sample(&field, Vec3::new(-1.01, 2.0, 4.0))
            .unwrap()
            .is_none()
    );
    assert!(sample(&field, Vec3::new(3.01, 2.0, 4.0)).unwrap().is_none());
}

#[test]
fn surface_query_visits_only_local_cells_that_can_contain_zero() {
    let field = LevelSet::from_vec(
        Vec3::ZERO,
        [5, 3, 2],
        Vec3::ONE,
        (0..30).map(|index| (index % 5) as f64 - 2.0).collect(),
    )
    .unwrap();
    let mut cells = Vec::new();
    query_surface_cells(
        &field,
        Aabb::new(Vec3::new(2.1, 0.1, 0.1), Vec3::new(2.9, 0.9, 0.9)).unwrap(),
        &mut cells,
    )
    .unwrap();
    assert!(!cells.is_empty());
    assert!(cells.len() <= 2);
    assert!(cells.iter().all(|cell| {
        cell.bounds
            .intersects(&Aabb::new(Vec3::new(2.1, 0.1, 0.1), Vec3::new(2.9, 0.9, 0.9)).unwrap())
    }));
}

#[test]
fn surface_query_respects_world_transform_and_returns_no_exterior_cells() {
    let field = LevelSet::from_vec(Vec3::ZERO, [2, 2, 2], Vec3::ONE, vec![0.0; 8]).unwrap();
    let rotation = Quat::from_axis_angle(Vec3::Z, std::f64::consts::FRAC_PI_2).unwrap();
    let transform = Transform::new(Vec3::new(5.0, 2.0, -1.0), rotation).unwrap();
    let mut cells = Vec::new();
    query_world_cells(
        &field,
        &transform,
        Aabb::new(Vec3::new(3.9, 2.9, -1.1), Vec3::new(4.1, 3.1, -0.9)).unwrap(),
        &mut cells,
    )
    .unwrap();
    assert_eq!(cells.len(), 1);
    query_world_cells(
        &field,
        &transform,
        Aabb::new(Vec3::splat(50.0), Vec3::splat(51.0)).unwrap(),
        &mut cells,
    )
    .unwrap();
    assert!(cells.is_empty());
}
