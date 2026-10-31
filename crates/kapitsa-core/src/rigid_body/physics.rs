use crate::mass::MassProperties;

/// Motion model used by a rigid body.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BodyType {
    Static,
    Kinematic,
    Dynamic,
}

/// Physical characteristics that determine a body's motion model.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BodyPhysics {
    pub(super) body_type: BodyType,
    pub(super) mass: Option<MassProperties>,
}

impl BodyPhysics {
    pub fn body_type(&self) -> BodyType {
        self.body_type
    }

    pub fn mass(&self) -> Option<MassProperties> {
        self.mass
    }
}
