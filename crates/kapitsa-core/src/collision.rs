mod config;
mod contact;
mod error;
mod event;

pub use config::CollisionConfig;
pub use contact::CollisionPair;
pub use error::{CollisionError, QueryError};
pub use event::{CollisionEvent, CollisionEventKind, ContactInfo};

#[cfg(test)]
mod tests;
