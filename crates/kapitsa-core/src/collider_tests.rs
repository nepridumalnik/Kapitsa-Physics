use std::error::Error;

use glam::{Affine3A, Quat, Vec3};

use crate::{
    aabb::{Aabb, AabbError},
    collider::{Collider, ColliderError, CollisionFilter},
    shape::{
        BoneInfluence, BoundsError, Cuboid, Plane, Shape, SkinnedTriangleMesh, Sphere, TriangleMesh,
    },
    transform::{Transform, TransformError},
};

fn sphere(radius: f32) -> Shape {
    Shape::Sphere(Sphere::new(radius).expect("valid sphere"))
}

fn transform(translation: Vec3, rotation: Quat) -> Transform {
    Transform::new(translation, rotation).expect("valid transform")
}

fn assert_vec3_close(actual: Vec3, expected: Vec3) {
    const EPSILON: f32 = 2.0e-5;

    assert!(
        (actual - expected).abs().max_element() <= EPSILON,
        "expected {expected:?}, got {actual:?}"
    );
}

#[test]
fn stores_shape_and_local_transform() {
    let shape = sphere(1.0);
    let local_transform = transform(Vec3::X, Quat::IDENTITY);
    let collider = Collider::new(shape, local_transform);

    assert!(matches!(collider.shape(), Shape::Sphere(_)));
    assert_eq!(collider.local_transform(), local_transform);
}

#[test]
fn collision_filter_defaults_to_first_layer_and_all_masks() {
    let filter = CollisionFilter::default();

    assert_eq!(filter.layers(), 1);
    assert_eq!(filter.mask(), u32::MAX);
}

#[test]
fn collision_filters_allow_interactions_on_the_same_layer() {
    let first = CollisionFilter::new(1, 1);
    let second = CollisionFilter::new(1, 1);

    assert!(first.allows(&second));
}

#[test]
fn collision_filters_allow_mutually_permitted_different_layers() {
    let first = CollisionFilter::new(0b0001, 0b0010);
    let second = CollisionFilter::new(0b0010, 0b0001);

    assert!(first.allows(&second));
}

#[test]
fn one_sided_collision_permission_is_rejected() {
    let first = CollisionFilter::new(0b0001, 0b0010);
    let second = CollisionFilter::new(0b0010, 0);

    assert!(!first.allows(&second));
}

#[test]
fn zero_collision_mask_disables_interactions() {
    let no_mask = CollisionFilter::new(1, 0);
    let default = CollisionFilter::default();

    assert!(!no_mask.allows(&default));
    assert!(!default.allows(&no_mask));
}

#[test]
fn collision_filters_support_multiple_layers() {
    let first = CollisionFilter::new(0b0011, 0b1100);
    let second = CollisionFilter::new(0b1100, 0b0011);

    assert!(first.allows(&second));
}

#[test]
fn collision_filter_interaction_is_symmetric() {
    let first = CollisionFilter::new(0b0101, 0b1010);
    let second = CollisionFilter::new(0b1010, 0b0101);

    assert_eq!(first.allows(&second), second.allows(&first));
}

#[test]
fn collider_filter_can_be_read_and_updated() {
    let mut collider = Collider::new(sphere(1.0), Transform::identity());
    let filter = CollisionFilter::new(0b0100, 0b1000);

    collider.set_filter(filter);

    assert_eq!(collider.filter(), filter);
}

#[test]
fn world_bounds_are_unchanged_when_filter_disables_collisions() {
    let mut collider = Collider::new(
        Shape::Cuboid(Cuboid::new(2.0, 4.0, 6.0).expect("valid cuboid")),
        Transform::identity(),
    );
    collider.set_filter(CollisionFilter::new(0, 0));

    assert_eq!(
        collider.world_bounds(&Transform::identity()),
        Ok(Some(
            Aabb::new(Vec3::new(-1.0, -2.0, -3.0), Vec3::new(1.0, 2.0, 3.0))
                .expect("valid expected bounds"),
        ))
    );
}

#[test]
fn identity_transforms_preserve_shape_bounds() {
    let collider = Collider::new(
        Shape::Cuboid(Cuboid::new(2.0, 4.0, 6.0).expect("valid cuboid")),
        Transform::identity(),
    );

    assert_eq!(
        collider.world_bounds(&Transform::identity()),
        Ok(Some(
            Aabb::new(Vec3::new(-1.0, -2.0, -3.0), Vec3::new(1.0, 2.0, 3.0))
                .expect("valid expected bounds"),
        ))
    );
}

#[test]
fn local_translation_moves_collider_bounds() {
    let collider = Collider::new(
        sphere(0.5),
        transform(Vec3::new(2.0, 0.0, 0.0), Quat::IDENTITY),
    );

    let bounds = collider
        .world_bounds(&Transform::identity())
        .expect("valid collider bounds")
        .expect("sphere has finite bounds");

    assert_vec3_close(bounds.center(), Vec3::new(2.0, 0.0, 0.0));
}

#[test]
fn parent_translation_moves_collider_bounds() {
    let collider = Collider::new(sphere(0.5), Transform::identity());
    let parent = transform(Vec3::new(0.0, 3.0, 0.0), Quat::IDENTITY);

    let bounds = collider
        .world_bounds(&parent)
        .expect("valid collider bounds")
        .expect("sphere has finite bounds");

    assert_vec3_close(bounds.center(), Vec3::new(0.0, 3.0, 0.0));
}

#[test]
fn parent_rotation_rotates_offset_collider() {
    let collider = Collider::new(sphere(0.5), transform(Vec3::X, Quat::IDENTITY));
    let parent = transform(
        Vec3::new(1.0, 2.0, 3.0),
        Quat::from_rotation_z(std::f32::consts::FRAC_PI_2),
    );

    let bounds = collider
        .world_bounds(&parent)
        .expect("valid collider bounds")
        .expect("sphere has finite bounds");

    assert_vec3_close(bounds.center(), Vec3::new(1.0, 3.0, 3.0));
}

#[test]
fn parent_and_local_rotations_are_composed() {
    let local_rotation = Quat::from_rotation_z(std::f32::consts::FRAC_PI_4);
    let parent_rotation = Quat::from_rotation_z(std::f32::consts::FRAC_PI_2);
    let collider = Collider::new(
        Shape::Cuboid(Cuboid::new(2.0, 4.0, 2.0).expect("valid cuboid")),
        transform(Vec3::new(1.0, 0.0, 0.0), local_rotation),
    );
    let parent = transform(Vec3::ZERO, parent_rotation);

    let bounds = collider
        .world_bounds(&parent)
        .expect("valid collider bounds")
        .expect("cuboid has finite bounds");
    let composed = transform(parent_rotation * Vec3::X, parent_rotation * local_rotation);
    let expected = Aabb::new(Vec3::new(-1.0, -2.0, -1.0), Vec3::new(1.0, 2.0, 1.0))
        .expect("valid cuboid bounds")
        .transformed(&composed)
        .expect("valid transformed bounds");

    assert_vec3_close(bounds.min(), expected.min());
    assert_vec3_close(bounds.max(), expected.max());
}

#[test]
fn colliders_can_have_distinct_local_transforms_for_one_parent() {
    let first = Collider::new(sphere(0.25), transform(Vec3::X, Quat::IDENTITY));
    let second = Collider::new(sphere(0.25), transform(Vec3::Y * 2.0, Quat::IDENTITY));
    let parent = transform(Vec3::new(5.0, 0.0, 0.0), Quat::IDENTITY);

    let first_bounds = first
        .world_bounds(&parent)
        .expect("valid collider bounds")
        .expect("sphere has finite bounds");
    let second_bounds = second
        .world_bounds(&parent)
        .expect("valid collider bounds")
        .expect("sphere has finite bounds");

    assert_vec3_close(first_bounds.center(), Vec3::new(6.0, 0.0, 0.0));
    assert_vec3_close(second_bounds.center(), Vec3::new(5.0, 2.0, 0.0));
}

#[test]
fn plane_has_no_finite_world_bounds() {
    let plane = Shape::Plane(Plane::new(0.0, 1.0, 0.0, 0.0).expect("valid plane"));
    let collider = Collider::new(plane, transform(Vec3::X, Quat::from_rotation_z(0.2)));
    let parent = transform(Vec3::Y, Quat::from_rotation_x(0.4));

    assert_eq!(collider.world_bounds(&parent), Ok(None));
}

#[test]
fn skinned_mesh_bounds_error_is_preserved() {
    let mesh =
        TriangleMesh::new(vec![Vec3::ZERO, Vec3::X, Vec3::Y], vec![[0, 1, 2]]).expect("valid mesh");
    let influences = (0..3)
        .map(|_| vec![BoneInfluence::new(0, 1.0).expect("valid influence")])
        .collect();
    let skinned_mesh = SkinnedTriangleMesh::new(
        mesh,
        vec![String::from("root")],
        influences,
        Affine3A::IDENTITY,
        vec![Affine3A::IDENTITY],
    )
    .expect("valid skinned mesh");
    let collider = Collider::new(
        Shape::SkinnedTriangleMesh(skinned_mesh),
        Transform::identity(),
    );

    assert_eq!(
        collider.world_bounds(&Transform::identity()),
        Err(ColliderError::Bounds(BoundsError::NotImplemented))
    );
}

#[test]
fn transform_overflow_is_preserved() {
    let collider = Collider::new(
        sphere(1.0),
        transform(Vec3::splat(f32::MAX), Quat::IDENTITY),
    );
    let parent = transform(Vec3::splat(f32::MAX), Quat::IDENTITY);

    assert_eq!(
        collider.world_bounds(&parent),
        Err(ColliderError::Transform(
            TransformError::CompositionOverflow
        ))
    );
}

#[test]
fn error_source_chain_preserves_bounds_and_aabb_errors() {
    let collider = Collider::new(sphere(f32::MAX), Transform::identity());
    let parent = transform(Vec3::splat(f32::MAX), Quat::IDENTITY);
    let error = collider
        .world_bounds(&parent)
        .expect_err("sphere bounds overflow at maximum translation");

    let bounds_error =
        Error::source(&error).and_then(|source| source.downcast_ref::<BoundsError>());
    assert_eq!(
        bounds_error,
        Some(&BoundsError::InvalidBounds(AabbError::TransformOverflow))
    );

    let aabb_error = bounds_error
        .and_then(Error::source)
        .and_then(|source| source.downcast_ref::<AabbError>());
    assert_eq!(aabb_error, Some(&AabbError::TransformOverflow));
}
