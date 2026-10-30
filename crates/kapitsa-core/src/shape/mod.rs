mod capsule;
mod convex;
mod cuboid;
mod cylinder;
mod height_field;
mod level_set;
mod ml_level_set;
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
pub use ml_level_set::{MlLevelSet, MlLevelSetBone};
pub use plane::Plane;
pub use skinned_triangle_mesh::{BoneInfluence, SkinnedTriangleMesh};
pub use sphere::Sphere;
pub use tapered_capsule::TaperedCapsule;
pub use tapered_cylinder::TaperedCylinder;
pub use triangle::Triangle;
pub use triangle_mesh::TriangleMesh;

pub enum Shape {
    // Primitives
    Sphere(Sphere),
    Cube(Cuboid),
    Plane(Plane),
    Capsule(Capsule),
    Cylinder(Cylinder),
    TaperedCapsule(TaperedCapsule),
    TaperedCylinder(TaperedCylinder),
    Triangle(Triangle),
    // Complex geometry
    Convex(Convex),
    TriangleMesh(TriangleMesh),
    HeightField(HeightField),
    LevelSet(LevelSet),
    MlLevelSet(MlLevelSet),
    SkinnedTriangleMesh(SkinnedTriangleMesh),
}
