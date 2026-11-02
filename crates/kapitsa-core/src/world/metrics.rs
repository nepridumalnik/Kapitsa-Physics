#[derive(Clone, Copy, Debug, Default)]
pub struct PhysicsStepMetrics {
    pub collision: crate::collision::CollisionStepMetrics,
    pub joints: crate::joint::diagnostics::JointSolverMetrics,
}
