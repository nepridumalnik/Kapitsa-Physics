use super::CollisionError;
use kapitsa_math::Vec3;

pub(super) fn closest_point_on_triangle(
    point: Vec3,
    vertices: [Vec3; 3],
) -> Result<Vec3, CollisionError> {
    let [a, b, c] = vertices;
    let ab = b.checked_sub(a).ok_or(CollisionError::ArithmeticOverflow)?;
    let ac = c.checked_sub(a).ok_or(CollisionError::ArithmeticOverflow)?;
    let ap = point
        .checked_sub(a)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let d1 = ab
        .checked_dot(ap)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let d2 = ac
        .checked_dot(ap)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    if d1 <= 0.0 && d2 <= 0.0 {
        return Ok(a);
    }

    let bp = point
        .checked_sub(b)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let d3 = ab
        .checked_dot(bp)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let d4 = ac
        .checked_dot(bp)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    if d3 >= 0.0 && d4 <= d3 {
        return Ok(b);
    }

    let vc = d1 * d4 - d3 * d2;
    if vc.is_finite() && vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return interpolate(a, ab, v);
    }

    let cp = point
        .checked_sub(c)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let d5 = ab
        .checked_dot(cp)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let d6 = ac
        .checked_dot(cp)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    if d6 >= 0.0 && d5 <= d6 {
        return Ok(c);
    }

    let vb = d5 * d2 - d1 * d6;
    if vb.is_finite() && vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return interpolate(a, ac, w);
    }

    let va = d3 * d6 - d5 * d4;
    if va.is_finite() && va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let edge = c.checked_sub(b).ok_or(CollisionError::ArithmeticOverflow)?;
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return interpolate(b, edge, w);
    }

    let denominator = va + vb + vc;
    if !denominator.is_finite() || denominator.abs() <= f64::MIN_POSITIVE {
        return Err(CollisionError::NonConvergent);
    }
    let inverse = 1.0 / denominator;
    let v = vb * inverse;
    let w = vc * inverse;
    let result = a + ab * v + ac * w;
    result
        .is_finite()
        .then_some(result)
        .ok_or(CollisionError::ArithmeticOverflow)
}

fn interpolate(origin: Vec3, edge: Vec3, amount: f64) -> Result<Vec3, CollisionError> {
    let result = origin
        .checked_add(
            edge.checked_mul_scalar(amount)
                .ok_or(CollisionError::ArithmeticOverflow)?,
        )
        .ok_or(CollisionError::ArithmeticOverflow)?;
    result
        .is_finite()
        .then_some(result)
        .ok_or(CollisionError::ArithmeticOverflow)
}
