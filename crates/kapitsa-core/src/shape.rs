//! Collision geometry in collider-local coordinates.
//!
//! Sphere and Cuboid are centered at the local origin. Other shapes may store
//! local endpoints, vertices, or a grid origin. A collider's transform places
//! this geometry in world space.

use kapitsa_math::{Vec3, numeric};

use crate::{
    aabb::{Aabb, AabbError},
    transform::Transform,
};

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
    pub(crate) fn is_convex(&self) -> bool {
        matches!(
            self,
            Self::Sphere(_)
                | Self::Cuboid(_)
                | Self::Capsule(_)
                | Self::Cylinder(_)
                | Self::TaperedCapsule(_)
                | Self::TaperedCylinder(_)
                | Self::Convex(_)
        )
    }

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
            Self::SkinnedTriangleMesh(shape) => shape.local_bounds().map(Some),
        }
    }

    /// Computes world-space bounds using the supplied rigid transform.
    ///
    /// `Ok(None)` means the shape has no finite bounds.
    pub fn world_bounds(&self, transform: &Transform) -> Result<Option<Aabb>, BoundsError> {
        if let Self::Sphere(sphere) = self {
            let center = transform.translation();
            let radius = sphere.radius();

            let min = |value: f64| numeric::round_down(value).ok_or(AabbError::TransformOverflow);
            let max = |value: f64| numeric::round_up(value).ok_or(AabbError::TransformOverflow);

            let bounds = Aabb::new(
                Vec3::new(
                    min(center.x - radius)?,
                    min(center.y - radius)?,
                    min(center.z - radius)?,
                ),
                Vec3::new(
                    max(center.x + radius)?,
                    max(center.y + radius)?,
                    max(center.z + radius)?,
                ),
            )?;

            return Ok(Some(bounds));
        }

        self.local_bounds()?
            .map(|bounds| bounds.transformed(transform).map_err(BoundsError::from))
            .transpose()
    }
}

#[cfg(test)]
mod tests;
