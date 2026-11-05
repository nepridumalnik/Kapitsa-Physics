use kapitsa_math::Vec3;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum RowDimension {
    Linear,
    Angular,
}

/// Scalar constraint row with generalized Jacobians for both connected bodies.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct ConstraintRow {
    /// Stable within its owning joint, independent of which other rows are active.
    pub(crate) key: u8,
    pub(crate) dimension: RowDimension,
    pub(crate) body_a: usize,
    pub(crate) body_b: usize,
    pub(crate) linear_a: Vec3,
    pub(crate) angular_a: Vec3,
    /// Angular row arm for body A used by the position projection.
    ///
    /// The velocity Jacobian must be the exact time derivative of the
    /// constraint coordinate, which for frame-derived (co-rotating) axes adds
    /// the `delta * (omega_a x axis)` term to body A's angular arm. Solving
    /// positions along that same arm lets large separations rotate the whole
    /// frame instead of moving the anchors, which is a near-null direction of
    /// the joint system and destroys Gauss-Seidel convergence. The position
    /// projection therefore follows the frozen-axis arm, a valid descent
    /// direction that keeps the anchors themselves responsible for closing
    /// positional gaps.
    pub(crate) position_angular_a: Vec3,
    pub(crate) linear_b: Vec3,
    pub(crate) angular_b: Vec3,
    pub(crate) error: f64,
    pub(crate) target_velocity: f64,
    /// Constant force along the constraint axis, in newtons or newton-meters.
    pub(crate) force_bias: f64,
    pub(crate) position_enabled: bool,
    pub(crate) compliance: f64,
    /// Viscous damping in N·s/m or N·m·s/rad.
    pub(crate) damping: f64,
    pub(crate) min_multiplier: f64,
    pub(crate) max_multiplier: f64,
    pub(crate) min_impulse: f64,
    pub(crate) max_impulse: f64,
    pub(crate) accumulated_multiplier: f64,
    pub(crate) accumulated_impulse: f64,
    /// Inverse effective mass cached while body poses and row Jacobians stay fixed.
    pub(crate) cached_velocity_effective_mass: f64,
}

impl ConstraintRow {
    pub(crate) fn is_valid(self, body_count: usize) -> bool {
        self.body_a != self.body_b
            && self.body_a < body_count
            && self.body_b < body_count
            && self.linear_a.is_finite()
            && self.angular_a.is_finite()
            && self.position_angular_a.is_finite()
            && self.linear_b.is_finite()
            && self.angular_b.is_finite()
            && self.error.is_finite()
            && self.target_velocity.is_finite()
            && self.force_bias.is_finite()
            && self.compliance.is_finite()
            && self.compliance >= 0.0
            && self.damping.is_finite()
            && self.damping >= 0.0
            && !self.min_multiplier.is_nan()
            && !self.max_multiplier.is_nan()
            && self.min_multiplier <= self.max_multiplier
            && !self.min_impulse.is_nan()
            && !self.max_impulse.is_nan()
            && self.min_impulse <= self.max_impulse
            && self.accumulated_multiplier.is_finite()
            && self.accumulated_impulse.is_finite()
            && self.cached_velocity_effective_mass.is_finite()
            && self.cached_velocity_effective_mass >= 0.0
    }
}
