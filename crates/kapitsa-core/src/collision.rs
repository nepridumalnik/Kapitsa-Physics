mod broad_phase;
mod config;
mod contact;
#[allow(dead_code)]
mod epa;
mod error;
mod event;
#[allow(dead_code)]
mod gjk;
#[allow(dead_code)]
mod height_field;
#[allow(dead_code)]
mod level_set;
#[allow(dead_code)]
mod manifold;
#[allow(dead_code)]
pub(crate) mod mesh;
#[allow(dead_code)]
mod narrow_phase;
pub(crate) mod spatial_index;
mod step;
#[allow(dead_code)]
mod support;
mod system;

pub use config::CollisionConfig;
pub use contact::CollisionPair;
pub use error::{CollisionError, QueryError};
pub use event::{CollisionEvent, CollisionEventKind, ContactInfo};

pub(crate) use broad_phase::CollisionProxy;
pub(crate) use step::StepContext;
pub(crate) use system::CollisionSystem;

#[cfg(test)]
mod tests;
