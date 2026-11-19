#![doc = include_str!("../README.md")]

pub mod aabb;
pub mod aerodynamics;
pub mod chunk;
pub mod cluster;
pub mod collider;
pub mod collision;
#[doc(hidden)]
pub mod command;
mod constraint;
#[doc(hidden)]
pub mod containers;
pub mod error;
pub mod execution;
pub mod gravity;
pub mod hydrodynamics;
pub mod joint;
pub mod mass;
pub mod material;
pub mod rigid_body;
pub mod shape;
pub mod transform;
pub mod world;

pub use error::{Error, ErrorKind};
pub use gravity::CentralGravity;
