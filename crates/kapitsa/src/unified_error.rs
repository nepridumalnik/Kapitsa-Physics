//! Conversions from runtime-specific failures into the unified engine error.

use kapitsa_core::{Error, ErrorKind};

use crate::runtime::{ConfigError, WorkerConfigError};

impl From<ConfigError> for Error {
    fn from(error: ConfigError) -> Self {
        Self::from_source(ErrorKind::Config, error)
    }
}

impl From<WorkerConfigError> for Error {
    fn from(error: WorkerConfigError) -> Self {
        Self::from_source(ErrorKind::Config, error)
    }
}

#[cfg(feature = "service")]
mod service_errors {
    use super::{Error, ErrorKind};
    use crate::runtime::{
        CollisionEventStreamError, JointBreakEventStreamError, PhysicsBatchResultStreamError,
        PhysicsClientError, PhysicsSubmitError, RayonPoolError, SimulationError,
    };

    impl From<PhysicsClientError> for Error {
        fn from(error: PhysicsClientError) -> Self {
            match error {
                PhysicsClientError::World(error) => {
                    Error::from(error).with_kind(ErrorKind::Runtime)
                }
                PhysicsClientError::Body(error) => Error::from(error).with_kind(ErrorKind::Runtime),
                PhysicsClientError::Cluster(error) => {
                    Error::from(error).with_kind(ErrorKind::Runtime)
                }
                PhysicsClientError::Query(error) => {
                    Error::from(error).with_kind(ErrorKind::Runtime)
                }
                PhysicsClientError::ForceSource(error) => {
                    Error::from(error).with_kind(ErrorKind::Runtime)
                }
                PhysicsClientError::StepFailedAfterApply { step, cause } => {
                    cause.with_kind(ErrorKind::Runtime).with_failed_step(step)
                }
                other => Error::from_source(ErrorKind::Runtime, other),
            }
        }
    }

    impl From<PhysicsSubmitError> for Error {
        fn from(error: PhysicsSubmitError) -> Self {
            Error::from_source(ErrorKind::Runtime, error)
        }
    }

    impl From<SimulationError> for Error {
        fn from(error: SimulationError) -> Self {
            match error {
                SimulationError::Physics(error) | SimulationError::TaskFailed(error) => error,
                SimulationError::InvalidConfig(error) => Error::from(error),
                other => Error::from_source(ErrorKind::Runtime, other),
            }
        }
    }

    impl From<RayonPoolError> for Error {
        fn from(error: RayonPoolError) -> Self {
            Error::from_source(ErrorKind::Runtime, error)
        }
    }

    impl From<CollisionEventStreamError> for Error {
        fn from(error: CollisionEventStreamError) -> Self {
            Error::from_source(ErrorKind::Runtime, error)
        }
    }

    impl From<JointBreakEventStreamError> for Error {
        fn from(error: JointBreakEventStreamError) -> Self {
            Error::from_source(ErrorKind::Runtime, error)
        }
    }

    impl From<PhysicsBatchResultStreamError> for Error {
        fn from(error: PhysicsBatchResultStreamError) -> Self {
            Error::from_source(ErrorKind::Runtime, error)
        }
    }
}

#[cfg(all(test, feature = "service"))]
mod tests {
    use kapitsa_core::{ErrorKind, world::StepError};

    use crate::runtime::PhysicsClientError;

    #[test]
    fn step_failure_keeps_attempt_and_typed_cause() {
        let error = kapitsa_core::Error::from(PhysicsClientError::StepFailedAfterApply {
            step: 7,
            cause: StepError::InvalidDeltaTime.into(),
        });
        assert_eq!(error.kind(), ErrorKind::Runtime);
        assert_eq!(error.failed_step(), Some(7));
        assert!(error.downcast_ref::<StepError>().is_some());
    }
}
