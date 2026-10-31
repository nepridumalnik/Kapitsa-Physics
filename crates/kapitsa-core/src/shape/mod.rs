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
    /// Computes finite local-space bounds when they are available.
    ///
    /// `Ok(None)` means the shape has no finite AABB. `Err` reports a bounds
    /// calculation error or a shape whose bounds are not implemented. In
    /// particular, `NotImplemented` does not mean that the shape has no collisions.
    pub fn local_bounds(&self) -> Result<Option<Aabb>, BoundsError> {
        match self {
            Self::Sphere(shape) => shape.local_bounds().map(Some),
            Self::Cuboid(shape) => shape.local_bounds().map(Some),
            Self::Plane(_) => Ok(None),
            Self::Capsule(shape) => shape.local_bounds().map(Some),
            Self::Cylinder(shape) => shape.local_bounds().map(Some),
            Self::TaperedCapsule(shape) => shape.local_bounds().map(Some),
            Self::TaperedCylinder(shape) => shape.local_bounds().map(Some),
            Self::Triangle(shape) => shape.local_bounds().map(Some),
            Self::Convex(shape) => shape.local_bounds().map(Some),
            Self::TriangleMesh(shape) => shape.local_bounds().map(Some),
            Self::HeightField(shape) => shape.local_bounds().map(Some),
            Self::LevelSet(shape) => shape.local_bounds().map(Some),
            Self::SkinnedTriangleMesh(_) => Err(BoundsError::NotImplemented),
        }
    }
}

#[cfg(test)]
mod tests;
