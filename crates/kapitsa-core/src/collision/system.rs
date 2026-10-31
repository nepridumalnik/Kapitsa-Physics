use super::{CollisionConfig, CollisionError, CollisionEvent, StepContext};
use crate::{rigid_body::BodyState, world::BodyId};

/// Owns reusable collision-step scratch and the last committed event batch.
pub(crate) struct CollisionSystem {
    config: CollisionConfig,
    pub(crate) context: StepContext,
    events: Vec<CollisionEvent>,
}

impl CollisionSystem {
    pub(crate) const fn new() -> Self {
        Self {
            config: CollisionConfig::default_const(),
            context: StepContext::new(),
            events: Vec::new(),
        }
    }

    pub(crate) fn config(&self) -> CollisionConfig {
        self.config
    }

    pub(crate) fn set_config(&mut self, config: CollisionConfig) -> Result<(), CollisionError> {
        config.validate()?;
        self.config = config;
        Ok(())
    }

    pub(crate) fn events(&self) -> &[CollisionEvent] {
        &self.events
    }

    pub(crate) fn begin_step(&mut self) {
        self.context.clear();
    }

    pub(crate) fn stage_prediction(&mut self, body_id: BodyId, state: BodyState) {
        self.context.predicted_states.push((body_id, state));
    }

    pub(crate) fn abort_step(&mut self) {
        self.context.clear();
    }

    pub(crate) fn commit_step(&mut self) {
        std::mem::swap(&mut self.events, &mut self.context.events);
        self.context.clear();
    }
}
