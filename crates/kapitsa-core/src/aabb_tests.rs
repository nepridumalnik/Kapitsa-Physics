use glam::Vec3;
use rstest::rstest;

use super::{Aabb, AabbError};

fn bounds(min: Vec3, max: Vec3) -> Aabb {
    Aabb::new(min, max).expect("valid AABB")
}

#[test]
fn stores_bounds() {
    let min = Vec3::new(-1.0, -2.0, -3.0);
    let max = Vec3::new(4.0, 5.0, 6.0);
    let aabb = bounds(min, max);

    assert_eq!(aabb.min(), min);
    assert_eq!(aabb.max(), max);
}

#[test]
fn accepts_degenerate_bounds() {
    let point = bounds(Vec3::ONE, Vec3::ONE);
    assert_eq!(point.size(), Vec3::ZERO);

    let plane = bounds(Vec3::ZERO, Vec3::new(2.0, 0.0, 4.0));
    assert_eq!(plane.size(), Vec3::new(2.0, 0.0, 4.0));
}

#[rstest]
#[case::inverted_x(Vec3::X, Vec3::ZERO)]
#[case::inverted_y(Vec3::Y, Vec3::ZERO)]
#[case::inverted_z(Vec3::Z, Vec3::ZERO)]
fn rejects_inverted_bounds(#[case] min: Vec3, #[case] max: Vec3) {
    assert_eq!(Aabb::new(min, max), Err(AabbError::InvertedBounds));
}

#[rstest]
#[case::nan_min(Vec3::new(f32::NAN, 0.0, 0.0), Vec3::ONE)]
#[case::nan_max(Vec3::ZERO, Vec3::new(1.0, f32::NAN, 1.0))]
#[case::positive_infinite_min(Vec3::new(f32::INFINITY, 0.0, 0.0), Vec3::ONE)]
#[case::negative_infinite_min(Vec3::new(f32::NEG_INFINITY, 0.0, 0.0), Vec3::ONE)]
#[case::positive_infinite_max(Vec3::ZERO, Vec3::new(1.0, f32::INFINITY, 1.0))]
#[case::negative_infinite_max(Vec3::ZERO, Vec3::new(1.0, f32::NEG_INFINITY, 1.0))]
fn rejects_non_finite_bounds(#[case] min: Vec3, #[case] max: Vec3) {
    assert_eq!(Aabb::new(min, max), Err(AabbError::NonFiniteBounds));
}

#[test]
fn computes_center_and_size() {
    let aabb = bounds(Vec3::new(-4.0, -2.0, -6.0), Vec3::new(2.0, 6.0, 10.0));

    assert_eq!(aabb.center(), Vec3::new(-1.0, 2.0, 2.0));
    assert_eq!(aabb.size(), Vec3::new(6.0, 8.0, 16.0));
}

#[test]
fn computes_center_without_intermediate_overflow() {
    let aabb = bounds(Vec3::splat(-f32::MAX), Vec3::splat(f32::MAX));

    assert_eq!(aabb.center(), Vec3::ZERO);
}

#[rstest]
#[case::inside(Vec3::ZERO, true)]
#[case::minimum(Vec3::new(-1.0, -2.0, -3.0), true)]
#[case::maximum(Vec3::new(1.0, 2.0, 3.0), true)]
#[case::on_face(Vec3::new(1.0, 0.0, 0.0), true)]
#[case::outside_x(Vec3::new(1.1, 0.0, 0.0), false)]
#[case::outside_y(Vec3::new(0.0, -2.1, 0.0), false)]
#[case::outside_z(Vec3::new(0.0, 0.0, 3.1), false)]
#[case::nan(Vec3::new(f32::NAN, 0.0, 0.0), false)]
fn contains_point(#[case] point: Vec3, #[case] expected: bool) {
    let aabb = bounds(Vec3::new(-1.0, -2.0, -3.0), Vec3::new(1.0, 2.0, 3.0));

    assert_eq!(aabb.contains(point), expected);
}

#[rstest]
#[case::identical(Vec3::splat(-1.0), Vec3::ONE, true)]
#[case::overlapping(Vec3::ZERO, Vec3::splat(2.0), true)]
#[case::contained(Vec3::splat(-0.5), Vec3::splat(0.5), true)]
#[case::containing(Vec3::splat(-2.0), Vec3::splat(2.0), true)]
#[case::touching_face(
    Vec3::new(1.0, -0.5, -0.5),
    Vec3::new(2.0, 0.5, 0.5),
    true
)]
#[case::touching_edge(
    Vec3::new(1.0, 1.0, -0.5),
    Vec3::new(2.0, 2.0, 0.5),
    true
)]
#[case::touching_corner(Vec3::ONE, Vec3::splat(2.0), true)]
#[case::separated_x(
    Vec3::new(1.1, -0.5, -0.5),
    Vec3::new(2.0, 0.5, 0.5),
    false
)]
#[case::separated_y(
    Vec3::new(-0.5, 1.1, -0.5),
    Vec3::new(0.5, 2.0, 0.5),
    false
)]
#[case::separated_z(
    Vec3::new(-0.5, -0.5, 1.1),
    Vec3::new(0.5, 0.5, 2.0),
    false
)]
fn detects_intersection(#[case] min: Vec3, #[case] max: Vec3, #[case] expected: bool) {
    let a = bounds(Vec3::splat(-1.0), Vec3::ONE);
    let b = bounds(min, max);

    assert_eq!(a.intersects(&b), expected);
    assert_eq!(b.intersects(&a), expected);
}

#[test]
fn union_encloses_both_bounds() {
    let a = bounds(Vec3::new(-2.0, 0.0, 1.0), Vec3::new(0.0, 2.0, 3.0));
    let b = bounds(Vec3::new(-1.0, -3.0, 2.0), Vec3::new(4.0, 1.0, 5.0));

    let expected = bounds(Vec3::new(-2.0, -3.0, 1.0), Vec3::new(4.0, 2.0, 5.0));

    assert_eq!(a.union(&b), expected);
    assert_eq!(b.union(&a), expected);
}

#[test]
fn union_with_contained_bounds_preserves_outer_bounds() {
    let outer = bounds(Vec3::splat(-2.0), Vec3::splat(2.0));
    let inner = bounds(Vec3::splat(-1.0), Vec3::ONE);

    assert_eq!(outer.union(&inner), outer);
    assert_eq!(inner.union(&outer), outer);
    assert_eq!(outer.union(&outer), outer);
}
