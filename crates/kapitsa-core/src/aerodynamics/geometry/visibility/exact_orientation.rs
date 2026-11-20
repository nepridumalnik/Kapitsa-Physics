//! Exact binary64 determinant accumulation for wide orientation predicates.
use super::{GeometryError, Vec3, components, scale};

// Products start at exponent -3222 (three smallest subnormals). Each of
// the 24 terms is below 2^3072, so their sum fits below 2^3077.
const MIN_EXPONENT: i32 = -3222;
const LIMBS: usize = 99;

// Each term places at most six payload words. Per payload, there is one
// initial limb write and at most one terminating carry write. Intermediate
// carry writes turn an all-ones limb into zero. Since only the initial and
// terminating writes can create all-ones limbs, at most two intermediate
// carry writes per payload are possible amortized across a determinant.
pub(super) const WORK: usize = 24 * 6 * 4 + 8 * LIMBS;
pub(super) const PROJECTED_WORK: usize = 18 * 6 * 4 + 8 * LIMBS;

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
        // Multilinearity cancels terms with two copies of d.
        Self::from_terms([
            ([a, b, c], false),
            ([d, b, c], true),
            ([a, d, c], true),
            ([a, b, d], true),
        ])
    }

    fn projected(
        start: [Factor; 3],
        end: [Factor; 3],
        point: [Factor; 3],
        direction: [Factor; 3],
    ) -> Self {
        // det(end-start, point-start, direction), expanded before subtraction.
        Self::from_terms([
            ([end, point, direction], false),
            ([start, point, direction], true),
            ([end, start, direction], true),
        ])
    }

    fn from_terms(terms: impl IntoIterator<Item = ([[Factor; 3]; 3], bool)>) -> Self {
        let mut sums = [[0; LIMBS]; 2];
        for ([a, b, c], negative) in terms {
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

    fn scaled(&self, leading: usize, common: i32) -> Result<f64, crate::Error> {
        // Round once to binary64's significand, or to its subnormal grid.
        let shift = leading
            .saturating_sub(52)
            .max((common - 1074).max(0) as usize);
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
        let magnitude = scale(top as f64, shift as i32 - common)?;
        Ok(if self.negative { -magnitude } else { magnitude })
    }
}

fn factors(point: Vec3) -> [Factor; 3] {
    components(point).map(Factor::new)
}

pub(super) fn area_fraction(points: [Vec3; 3]) -> Result<f64, crate::Error> {
    let [a, b, c] = points.map(|point| factors(Vec3::new(point.x, point.y, 0.0)));
    let determinant = Determinant::new([a, b, c, factors(Vec3::Z)]);
    if let Some(leading) = determinant.leading_bit() {
        // Each integer bit represents a power of two relative to MIN_EXPONENT.
        Ok(determinant.scaled(leading, -MIN_EXPONENT)?.abs())
    } else {
        Ok(0.0)
    }
}

pub(super) fn projected_sign(
    start: Vec3,
    end: Vec3,
    point: Vec3,
    direction: Vec3,
) -> Result<f64, crate::Error> {
    if [start, end, point, direction]
        .iter()
        .any(|point| !point.is_finite())
    {
        return Err(GeometryError::ArithmeticOverflow.into());
    }
    let determinant = Determinant::projected(
        factors(start),
        factors(end),
        factors(point),
        factors(direction),
    );
    Ok(match (determinant.leading_bit(), determinant.negative) {
        (None, _) => 0.0,
        (Some(_), true) => -1.0,
        (Some(_), false) => 1.0,
    })
}

pub(super) fn projected_coefficient(
    [start, end, point]: [Vec3; 3],
    direction: Vec3,
    scale_exponent: i32,
) -> Result<f64, crate::Error> {
    if [start, end, point, direction]
        .iter()
        .any(|point| !point.is_finite())
    {
        return Err(GeometryError::ArithmeticOverflow.into());
    }
    let [start, end, point, direction] = [start, end, point, direction].map(factors);
    let determinant = Determinant::projected(start, end, point, direction);
    if let Some(leading) = determinant.leading_bit() {
        determinant.scaled(leading, -MIN_EXPONENT - scale_exponent)
    } else {
        Ok(0.0)
    }
}

pub(super) fn projected_coefficients(
    start: Vec3,
    end: Vec3,
    points: [Vec3; 3],
    direction: Vec3,
) -> Result<[f64; 3], crate::Error> {
    if [start, end, direction]
        .iter()
        .chain(&points)
        .any(|point| !point.is_finite())
    {
        return Err(GeometryError::ArithmeticOverflow.into());
    }
    let [start, end, direction] = [start, end, direction].map(factors);
    scaled_values(points.map(|point| Determinant::projected(start, end, factors(point), direction)))
}

pub(super) fn orientations(basis: [Vec3; 3], points: [Vec3; 3]) -> Result<[f64; 3], crate::Error> {
    if basis.iter().chain(&points).any(|point| !point.is_finite()) {
        return Err(GeometryError::ArithmeticOverflow.into());
    }
    let [a, b, c] = basis.map(factors);
    scaled_values(points.map(|point| Determinant::new([a, b, c, factors(point)])))
}

fn scaled_values(determinants: [Determinant; 3]) -> Result<[f64; 3], crate::Error> {
    let leading = determinants.each_ref().map(Determinant::leading_bit);
    let Some(common) = leading.iter().flatten().copied().max() else {
        return Ok([0.0; 3]);
    };
    let mut result = [0.0; 3];
    for ((result, determinant), leading) in result.iter_mut().zip(determinants).zip(leading) {
        if let Some(leading) = leading {
            *result = determinant.scaled(leading, common as i32)?;
        }
    }
    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn projected_coefficient_uses_the_requested_binary_scale() -> Result<(), crate::Error> {
        let points = [Vec3::ZERO, Vec3::X, Vec3::Y * 0.25];
        for exponent in [-1000, 0, 1000] {
            assert_eq!(
                projected_coefficient(points, Vec3::Z, exponent)?,
                2.0_f64.powi(exponent - 2)
            );
        }
        Ok(())
    }

    #[test]
    fn projected_determinants_preserve_displacement_lost_by_subtraction() -> Result<(), crate::Error>
    {
        let start = Vec3::X * 2.0_f64.powi(-500);
        let end = Vec3::X + Vec3::Y;
        let point = end * 2.0;
        assert_eq!((end - start).cross(point - start), Vec3::ZERO);
        assert_eq!(projected_sign(start, end, point, Vec3::Z)?, -1.0);
        assert_eq!(
            projected_coefficients(start, end, [point, end, Vec3::ZERO], Vec3::Z)?,
            [-1.0, 0.0, 1.0],
        );
        Ok(())
    }

    #[test]
    fn projected_determinants_accept_overflowing_endpoint_difference() -> Result<(), crate::Error> {
        let start = Vec3::X * f64::MAX;
        let end = -start;
        assert!(!(end - start).is_finite());
        let values =
            projected_coefficients(start, end, [Vec3::Y, -Vec3::Y * 0.5, Vec3::ZERO], Vec3::Z)?;
        assert!(values[0] < 0.0);
        assert_eq!(values[1], -values[0] / 2.0);
        assert_eq!(values[2], 0.0);
        Ok(())
    }

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
