use super::row::ConstraintRow;

/// Reusable rows for one solver island, retained between physics steps.
#[derive(Default)]
pub(crate) struct JointWorkspace {
    rows: Vec<ConstraintRow>,
    prepared_rows: usize,
}

impl JointWorkspace {
    pub(crate) const fn new() -> Self {
        Self {
            rows: Vec::new(),
            prepared_rows: 0,
        }
    }

    pub(crate) fn begin_step(&mut self) {
        self.prepared_rows = 0;
        for row in &mut self.rows {
            row.accumulated_multiplier = 0.0;
            row.accumulated_impulse = 0.0;
        }
    }

    pub(crate) fn clear(&mut self) {
        self.rows.clear();
        self.prepared_rows = 0;
    }

    pub(crate) fn begin_prepare(&mut self) {
        self.prepared_rows = 0;
    }

    pub(crate) fn push(&mut self, mut row: ConstraintRow) {
        if let Some(previous) = self.rows.get(self.prepared_rows) {
            row.accumulated_multiplier = previous.accumulated_multiplier;
            row.accumulated_impulse = previous.accumulated_impulse;
            self.rows[self.prepared_rows] = row;
        } else {
            self.rows.push(row);
        }
        self.prepared_rows += 1;
    }

    pub(crate) fn finish_prepare(&mut self) {
        self.rows.truncate(self.prepared_rows);
    }

    pub(crate) fn rows(&self) -> &[ConstraintRow] {
        &self.rows
    }

    pub(crate) fn rows_mut(&mut self) -> &mut [ConstraintRow] {
        &mut self.rows
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn retained_capacity_bytes(&self) -> usize {
        self.rows.capacity() * std::mem::size_of::<ConstraintRow>()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use kapitsa_math::Vec3;

    #[test]
    fn clear_reuses_row_capacity() {
        let mut workspace = JointWorkspace::new();
        workspace.push(ConstraintRow {
            body_a: 0,
            body_b: 1,
            linear_a: -Vec3::X,
            angular_a: Vec3::ZERO,
            linear_b: Vec3::X,
            angular_b: Vec3::ZERO,
            error: 0.0,
            target_velocity: 0.0,
            compliance: 0.0,
            damping: 0.0,
            min_multiplier: f64::NEG_INFINITY,
            max_multiplier: f64::INFINITY,
            min_impulse: f64::NEG_INFINITY,
            max_impulse: f64::INFINITY,
            accumulated_multiplier: 0.0,
            accumulated_impulse: 0.0,
        });
        let capacity = workspace.rows.capacity();
        assert!(capacity > 0);

        workspace.clear();

        assert!(workspace.rows.is_empty());
        assert_eq!(workspace.rows.capacity(), capacity);
    }
}
