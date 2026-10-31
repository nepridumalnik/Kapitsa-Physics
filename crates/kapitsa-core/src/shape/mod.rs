//! Collision geometry in collider-local coordinates.
//!
//! Sphere and Cuboid are centered at the local origin. Other shapes may store
//! local endpoints, vertices, or a grid origin. A collider's transform places
//! this geometry in world space.

use crate::aabb::Aabb;

pub mod error;
pub use error::{BoundsError, ShapeError};

mod capsule;
mod convex;
mod cuboid;
mod cylinder;
mod height_field;
mod level_set;
mod plane;
mod skinned_triangle_mesh;
mod sphere;
mod tapered_capsule;
mod tapered_cylinder;
mod triangle;
mod triangle_mesh;

pub use capsule::Capsule;
pub use convex::Convex;
pub use cuboid::Cuboid;
pub use cylinder::Cylinder;
pub use height_field::HeightField;
pub use level_set::LevelSet;
pub use plane::Plane;
pub use skinned_triangle_mesh::{BoneInfluence, SkinnedTriangleMesh};
pub use sphere::Sphere;
pub use tapered_capsule::TaperedCapsule;
pub use tapered_cylinder::TaperedCylinder;
pub use triangle::Triangle;
pub use triangle_mesh::TriangleMesh;

pub enum Shape {
    Sphere(Sphere),
    Cuboid(Cuboid),
    Plane(Plane),
    Capsule(Capsule),
    Cylinder(Cylinder),
    TaperedCapsule(TaperedCapsule),
    TaperedCylinder(TaperedCylinder),
    Triangle(Triangle),
    Convex(Convex),
    TriangleMesh(TriangleMesh),
    HeightField(HeightField),
    LevelSet(LevelSet),
    SkinnedTriangleMesh(SkinnedTriangleMesh),
}

impl Shape {
    /// Returns finite local-space bounds when they are available.
    ///
    /// `None` means that finite bounds are unavailable; it does not mean that
    /// the shape has no collisions.
    pub fn local_bounds(&self) -> Option<Aabb> {
        match self {
            Self::Sphere(shape) => shape.local_bounds(),
            Self::Cuboid(shape) => shape.local_bounds(),
            Self::Plane(_) => None,
            Self::Capsule(shape) => shape.local_bounds(),
            Self::Cylinder(shape) => shape.local_bounds(),
            Self::TaperedCapsule(shape) => shape.local_bounds(),
            Self::TaperedCylinder(shape) => shape.local_bounds(),
            Self::Triangle(shape) => shape.local_bounds(),
            Self::Convex(shape) => shape.local_bounds(),
            Self::TriangleMesh(shape) => shape.local_bounds(),
            Self::HeightField(shape) => shape.local_bounds(),
            Self::LevelSet(shape) => shape.local_bounds(),
            Self::SkinnedTriangleMesh(_) => {
                // TODO: Compute bounds from the current skinned vertices after skinning exists.
                None
            }
        }
    }
}

#[cfg(test)]
mod tests;
