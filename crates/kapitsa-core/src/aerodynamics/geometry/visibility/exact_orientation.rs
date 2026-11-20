//! Exact binary64 determinant accumulation for wide orientation predicates.
use super::{GeometryError, Vec3, components, scale};

// Products start at exponent -3222 (three smallest subnormals). Each of
// the 24 terms is below 2^3072, so their sum fits below 2^3077.
const MIN_EXPONENT: i32 = -3222;
const LIMBS: usize = 99;

// Charges one fixed-size determinant accumulation, including limb scans
// and carry propagation on both signs of the sum.
pub(super) const WORK: usize = (24 * 6 + 8) * LIMBS;

#[derive(Clone, Copy)]
struct Factor {
    mantissa: u64,
    exponent: i32,
    negative: bool,
}

impl Factor {
    fn new(value: f64) -> Self {
        let bits = value.to_bits();
        let biased = ((bits >> 52) & 0x7ff) as i32;
        let fraction = bits & ((1 << 52) - 1);
        Self {
            mantissa: fraction | if biased == 0 { 0 } else { 1 << 52 },
            exponent: biased.max(1) - 1075,
            negative: value.is_sign_negative(),
        }
    }
}

type Limbs = [u64; LIMBS];

fn add_word(sum: &mut Limbs, start: usize, mut word: u64) {
    for limb in &mut sum[start..] {
        if word == 0 {
            break;
        }
        let (value, carry) = limb.overflowing_add(word);
        *limb = value;
        word = u64::from(carry);
    }
    debug_assert_eq!(word, 0);
}

fn add_product(sums: &mut [Limbs; 2], [a, b, c]: [Factor; 3], negative: bool) {
    if a.mantissa == 0 || b.mantissa == 0 || c.mantissa == 0 {
        return;
    }
    let sum = &mut sums[usize::from(negative ^ a.negative ^ b.negative ^ c.negative)];
    let ab = u128::from(a.mantissa) * u128::from(b.mantissa);
    let low = u128::from(ab as u64) * u128::from(c.mantissa);
    let high = (ab >> 64) * u128::from(c.mantissa) + (low >> 64);
    let shift = (a.exponent + b.exponent + c.exponent - MIN_EXPONENT) as usize;
    let start = shift / 64;
    let offset = shift % 64;
    for (i, word) in [low as u64, high as u64, (high >> 64) as u64]
        .into_iter()
        .enumerate()
    {
        add_word(sum, start + i, word << offset);
        if offset != 0 {
            add_word(sum, start + i + 1, word >> (64 - offset));
        }
    }
}

struct Determinant {
    magnitude: Limbs,
    negative: bool,
}

impl Determinant {
    fn new([a, b, c, d]: [[Factor; 3]; 4]) -> Self {
        let mut sums = [[0; LIMBS]; 2];
        // det(a-d, b-d, c-d) expanded by multilinearity. Terms containing
        // two copies of d cancel, leaving four signed 3x3 determinants.
        for ([a, b, c], negative) in [
            ([a, b, c], false),
            ([d, b, c], true),
            ([a, d, c], true),
            ([a, b, d], true),
        ] {
            for (i, x) in a.into_iter().enumerate() {
                let j = (i + 1) % 3;
                let k = (i + 2) % 3;
                add_product(&mut sums, [x, b[j], c[k]], negative);
                add_product(&mut sums, [x, b[k], c[j]], !negative);
            }
        }
        let [positive, negative] = sums;
        let sign = positive.iter().rev().cmp(negative.iter().rev()).is_lt();
        let (mut magnitude, smaller) = if sign {
            (negative, positive)
        } else {
            (positive, negative)
        };
        let mut borrow = false;
        for (word, subtrahend) in magnitude.iter_mut().zip(smaller) {
            let (difference, underflow) = word.overflowing_sub(subtrahend);
            let (difference, carry) = difference.overflowing_sub(u64::from(borrow));
            *word = difference;
            borrow = underflow || carry;
        }
        debug_assert!(!borrow);
        Self {
            magnitude,
            negative: sign,
        }
    }

    fn leading_bit(&self) -> Option<usize> {
        let limb = self.magnitude.iter().rposition(|&word| word != 0)?;
        Some(limb * 64 + 63 - self.magnitude[limb].leading_zeros() as usize)
    }

    fn scaled(&self, leading: usize, common: usize) -> Result<f64, crate::Error> {
        // Round once to binary64's significand, or to its subnormal grid.
        let shift = leading.saturating_sub(52).max(common.saturating_sub(1074));
        let limb = shift / 64;
        let offset = shift % 64;
        let mut top = self.magnitude[limb] >> offset;
        if offset != 0 {
            top |= self.magnitude.get(limb + 1).copied().unwrap_or(0) << (64 - offset);
        }
        if shift != 0 {
            let half = shift - 1;
            let limb = half / 64;
            let bit = 1 << (half % 64);
            let round_bit = self.magnitude[limb] & bit != 0;
            let sticky = self.magnitude[limb] & (bit - 1) != 0
                || self.magnitude[..limb].iter().any(|&word| word != 0);
            top += u64::from(round_bit && (sticky || top & 1 != 0));
        }
        if top == 0 {
            return Err(GeometryError::PrecisionLoss.into());
        }
        let magnitude = scale(top as f64, shift as i32 - common as i32)?;
        Ok(if self.negative { -magnitude } else { magnitude })
    }
}

fn factors(point: Vec3) -> [Factor; 3] {
    components(point).map(Factor::new)
}

pub(super) fn sign(basis: [Vec3; 3], point: Vec3) -> Result<f64, crate::Error> {
    if !point.is_finite() || basis.iter().any(|point| !point.is_finite()) {
        return Err(GeometryError::ArithmeticOverflow.into());
    }
    let [a, b, c] = basis.map(factors);
    let determinant = Determinant::new([a, b, c, factors(point)]);
    Ok(match (determinant.leading_bit(), determinant.negative) {
        (None, _) => 0.0,
        (Some(_), true) => -1.0,
        (Some(_), false) => 1.0,
    })
}

pub(super) fn orientations(basis: [Vec3; 3], points: [Vec3; 3]) -> Result<[f64; 3], crate::Error> {
    if basis.iter().chain(&points).any(|point| !point.is_finite()) {
        return Err(GeometryError::ArithmeticOverflow.into());
    }
    let [a, b, c] = basis.map(factors);
    let determinants = points.map(|point| Determinant::new([a, b, c, factors(point)]));
    let leading = determinants.each_ref().map(Determinant::leading_bit);
    let Some(common) = leading.iter().flatten().copied().max() else {
        return Ok([0.0; 3]);
    };
    let mut result = [0.0; 3];
    for ((result, determinant), leading) in result.iter_mut().zip(determinants).zip(leading) {
        if let Some(leading) = leading {
            *result = determinant.scaled(leading, common)?;
        }
    }
    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn determinants_share_scale_across_wide_products() -> Result<(), crate::Error> {
        let maximum = f64::MAX;
        let minimum = f64::from_bits(1);
        let blocker = [Vec3::ZERO, Vec3::X * maximum, Vec3::Y * minimum];
        let source = [Vec3::Z * maximum, Vec3::Z * (-maximum / 2.0), Vec3::X];
        let values = orientations(blocker, source)?;
        assert!(values[0] < 0.0);
        assert_eq!(values[1], -values[0] / 2.0);
        assert_eq!(values[2], 0.0);
        assert_eq!(
            orientations([blocker[0], blocker[2], blocker[1]], source)?,
            values.map(|v| -v)
        );
        Ok(())
    }

    #[test]
    fn determinant_conversion_preserves_smallest_subnormal_or_errors() -> Result<(), crate::Error> {
        let minimum = f64::from_bits(1);
        assert_eq!(
            orientations(
                [Vec3::ZERO, Vec3::X, Vec3::Y],
                [Vec3::Z, Vec3::Z * minimum, Vec3::ZERO]
            )?,
            [-1.0, -minimum, 0.0],
        );
        assert_eq!(
            orientations(
                [Vec3::ZERO, Vec3::X, Vec3::Y],
                [Vec3::Z * 2.0, Vec3::Z * minimum, Vec3::ZERO]
            ),
            Err(GeometryError::PrecisionLoss.into()),
        );
        Ok(())
    }

    #[test]
    fn expanded_determinant_retains_depth_after_large_terms_cancel() -> Result<(), crate::Error> {
        let translation = 2.0_f64.powi(100);
        let t = Vec3::splat(translation);
        let edge = 2.0_f64.powi(50);
        let blocker = [t, t + Vec3::X * edge, t + Vec3::Y * edge];
        let values = orientations(blocker, [t + Vec3::Z * edge, t - Vec3::Z * edge, t])?;
        assert_eq!(values, [-1.0, 1.0, 0.0]);
        Ok(())
    }
}
