mod broad_phase;
mod config;
mod contact;
mod error;
mod event;
pub(crate) mod spatial_index;
mod step;
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
