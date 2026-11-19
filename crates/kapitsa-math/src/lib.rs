#![forbid(unsafe_code)]
// Documentation debt: `missing_docs` is intentionally exempted crate-wide here.
// The published `kapitsa` facade enforces `deny(missing_docs)`.
#![allow(missing_docs)]

mod affine;
mod matrix;
pub mod numeric;
mod quaternion;
mod vector;
mod vector2;

pub use affine::Affine3;
pub use matrix::{Cholesky3, CholeskyError, Mat3};
pub use quaternion::Quat;
pub use vector::Vec3;
pub use vector2::Vec2;
