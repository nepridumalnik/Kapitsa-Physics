/// Adds finite values, returning `None` if the result is not representable.
pub fn checked_add(left: f64, right: f64) -> Option<f64> {
    checked_result(left + right, left, right)
}

/// Subtracts finite values, returning `None` if the result is not representable.
pub fn checked_sub(left: f64, right: f64) -> Option<f64> {
    checked_result(left - right, left, right)
}

/// Multiplies finite values, returning `None` if the result is not representable.
pub fn checked_mul(left: f64, right: f64) -> Option<f64> {
    checked_result(left * right, left, right)
}

/// Divides finite values, returning `None` for zero divisors or non-finite results.
pub fn checked_div(numerator: f64, denominator: f64) -> Option<f64> {
    checked_result(numerator / denominator, numerator, denominator)
}

/// Returns a forward-error bound for `operation_count` floating-point
/// operations on an intermediate magnitude of `scale`.
///
/// This uses `gamma_n = n * EPSILON / (1 - n * EPSILON)`, scaled by the
/// caller-provided magnitude so callers can choose a scale meaningful to the
/// operation they are checking.
pub fn roundoff_bound(scale: f64, operation_count: u32) -> Option<f64> {
    if !scale.is_finite() || scale < 0.0 {
        return None;
    }

    let product = f64::from(operation_count) * f64::EPSILON;
    if product >= 1.0 {
        return None;
    }
    Some(scale * (product / (1.0 - product)))
}

/// Checks whether two finite values differ by no more than their calculated
/// floating-point roundoff bound.
pub fn within_roundoff(left: f64, right: f64, scale: f64, operation_count: u32) -> bool {
    if !left.is_finite() || !right.is_finite() {
        return false;
    }
    let Some(bound) = roundoff_bound(scale, operation_count) else {
        return false;
    };
    (left - right).abs() <= bound
}

/// Rounds a finite value down by one representable `f64` step.
///
/// Returns `None` when rounding would leave the finite `f64` range.
pub fn round_down(value: f64) -> Option<f64> {
    if !value.is_finite() {
        return None;
    }
    let rounded = value.next_down();
    rounded.is_finite().then_some(rounded)
}

/// Rounds a finite value up by one representable `f64` step.
///
/// Returns `None` when rounding would leave the finite `f64` range.
pub fn round_up(value: f64) -> Option<f64> {
    if !value.is_finite() {
        return None;
    }
    let rounded = value.next_up();
    rounded.is_finite().then_some(rounded)
}

fn checked_result(result: f64, left: f64, right: f64) -> Option<f64> {
    (left.is_finite() && right.is_finite() && result.is_finite()).then_some(result)
}

#[cfg(test)]
mod tests {
    use super::{
        checked_add, checked_div, checked_mul, checked_sub, round_down, round_up, roundoff_bound,
        within_roundoff,
    };

    #[test]
    fn checked_arithmetic_accepts_finite_results() {
        assert_eq!(checked_add(1.0, 2.0), Some(3.0));
        assert_eq!(checked_sub(1.0, 2.0), Some(-1.0));
        assert_eq!(checked_mul(2.0, 3.0), Some(6.0));
        assert_eq!(checked_div(6.0, 2.0), Some(3.0));
        assert_eq!(checked_mul(f64::MIN_POSITIVE, f64::MIN_POSITIVE), Some(0.0));
    }

    #[test]
    fn checked_arithmetic_rejects_invalid_inputs_and_overflow() {
        assert_eq!(checked_add(f64::MAX, f64::MAX), None);
        assert_eq!(checked_sub(f64::NEG_INFINITY, 1.0), None);
        assert_eq!(checked_mul(f64::MAX, 2.0), None);
        assert_eq!(checked_div(1.0, 0.0), None);
        assert_eq!(checked_div(f64::NAN, 1.0), None);
    }

    #[test]
    fn roundoff_bound_scales_with_data_and_operation_count() {
        let unit = roundoff_bound(1.0, 8).expect("finite scale");
        let large = roundoff_bound(1e100, 8).expect("finite scale");
        let more_operations = roundoff_bound(1.0, 16).expect("finite scale");

        assert!(unit > 0.0);
        assert!((large / unit - 1e100).abs() / 1e100 < 1e-14);
        assert!(more_operations > unit);
        assert_eq!(roundoff_bound(f64::INFINITY, 4), None);
        assert_eq!(roundoff_bound(-1.0, 4), None);
    }

    #[test]
    fn within_roundoff_handles_scale_and_invalid_data() {
        assert!(within_roundoff(1.0, 1.0 + 2.0 * f64::EPSILON, 1.0, 16));
        assert!(!within_roundoff(1.0, 1.0 + 1e-8, 1.0, 16));
        assert!(!within_roundoff(f64::NAN, 0.0, 1.0, 16));
    }

    #[test]
    fn rounds_bounds_outward_and_detects_finite_range_edges() {
        let value = 1.25;
        let lower = round_down(value).expect("finite lower bound");
        let upper = round_up(value).expect("finite upper bound");

        assert!(lower < value);
        assert!(upper > value);
        assert_eq!(round_down(f64::NEG_INFINITY), None);
        assert_eq!(round_up(f64::MAX), None);
    }
}
