use kapitsa_math::Vec3;

/// Scalar constraint row with generalized Jacobians for both connected bodies.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct ConstraintRow {
    pub(crate) body_a: usize,
    pub(crate) body_b: usize,
    pub(crate) linear_a: Vec3,
    pub(crate) angular_a: Vec3,
    pub(crate) linear_b: Vec3,
    pub(crate) angular_b: Vec3,
    pub(crate) error: f64,
    pub(crate) target_velocity: f64,
    pub(crate) compliance: f64,
    /// Viscous damping in N·s/m or N·m·s/rad.
    pub(crate) damping: f64,
    pub(crate) min_multiplier: f64,
    pub(crate) max_multiplier: f64,
    pub(crate) min_impulse: f64,
    pub(crate) max_impulse: f64,
    pub(crate) accumulated_multiplier: f64,
    pub(crate) accumulated_impulse: f64,
}

impl ConstraintRow {
    pub(crate) fn is_valid(self, body_count: usize) -> bool {
        self.body_a != self.body_b
            && self.body_a < body_count
            && self.body_b < body_count
            && self.linear_a.is_finite()
            && self.angular_a.is_finite()
            && self.linear_b.is_finite()
            && self.angular_b.is_finite()
            && self.error.is_finite()
            && self.target_velocity.is_finite()
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
    }
}
