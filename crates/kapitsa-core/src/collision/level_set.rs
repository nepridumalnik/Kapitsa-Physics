use crate::{aabb::Aabb, shape::LevelSet, transform::Transform};
use kapitsa_math::Vec3;

use super::CollisionError;

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct LevelSetSample {
    pub(crate) value: f64,
    pub(crate) gradient: Vec3,
    pub(crate) normal: Option<Vec3>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct LevelSetCell {
    pub(crate) index: u64,
    pub(crate) bounds: Aabb,
}

pub(crate) fn sample(
    level_set: &LevelSet,
    point: Vec3,
) -> Result<Option<LevelSetSample>, CollisionError> {
    let origin = level_set.origin();
    let spacing = level_set.grid_spacing();
    let dimensions = level_set.dimensions();
    let last_node = origin
        + spacing
            * Vec3::new(
                (dimensions[0] - 1) as f64,
                (dimensions[1] - 1) as f64,
                (dimensions[2] - 1) as f64,
            );
    if !point.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }
    if point.x < origin.x
        || point.y < origin.y
        || point.z < origin.z
        || point.x > last_node.x
        || point.y > last_node.y
        || point.z > last_node.z
    {
        return Ok(None);
    }

    let (x, tx) = locate_axis(point.x, origin.x, spacing.x, dimensions[0])?;
    let (y, ty) = locate_axis(point.y, origin.y, spacing.y, dimensions[1])?;
    let (z, tz) = locate_axis(point.z, origin.z, spacing.z, dimensions[2])?;
    let c000 = value_at(level_set, x, y, z)?;
    let c100 = value_at(level_set, x + 1, y, z)?;
    let c010 = value_at(level_set, x, y + 1, z)?;
    let c110 = value_at(level_set, x + 1, y + 1, z)?;
    let c001 = value_at(level_set, x, y, z + 1)?;
    let c101 = value_at(level_set, x + 1, y, z + 1)?;
    let c011 = value_at(level_set, x, y + 1, z + 1)?;
    let c111 = value_at(level_set, x + 1, y + 1, z + 1)?;

    let x00 = lerp(c000, c100, tx)?;
    let x10 = lerp(c010, c110, tx)?;
    let x01 = lerp(c001, c101, tx)?;
    let x11 = lerp(c011, c111, tx)?;
    let y0 = lerp(x00, x10, ty)?;
    let y1 = lerp(x01, x11, ty)?;
    let value = lerp(y0, y1, tz)?;

    let dx00 = checked_difference(c100, c000)? / spacing.x;
    let dx10 = checked_difference(c110, c010)? / spacing.x;
    let dx01 = checked_difference(c101, c001)? / spacing.x;
    let dx11 = checked_difference(c111, c011)? / spacing.x;
    let dy00 = checked_difference(c010, c000)? / spacing.y;
    let dy10 = checked_difference(c110, c100)? / spacing.y;
    let dy01 = checked_difference(c011, c001)? / spacing.y;
    let dy11 = checked_difference(c111, c101)? / spacing.y;
    let dz00 = checked_difference(c001, c000)? / spacing.z;
    let dz10 = checked_difference(c101, c100)? / spacing.z;
    let dz01 = checked_difference(c011, c010)? / spacing.z;
    let dz11 = checked_difference(c111, c110)? / spacing.z;
    let gradient = Vec3::new(
        lerp(lerp(dx00, dx10, ty)?, lerp(dx01, dx11, ty)?, tz)?,
        lerp(lerp(dy00, dy10, tx)?, lerp(dy01, dy11, tx)?, tz)?,
        lerp(lerp(dz00, dz10, tx)?, lerp(dz01, dz11, tx)?, ty)?,
    );
    if !gradient.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }
    Ok(Some(LevelSetSample {
        value,
        gradient,
        normal: gradient.try_normalize(),
    }))
}

pub(crate) fn query_surface_cells(
    level_set: &LevelSet,
    local_bounds: Aabb,
    output: &mut Vec<LevelSetCell>,
) -> Result<(), CollisionError> {
    output.clear();
    let dimensions = level_set.dimensions();
    let origin = level_set.origin();
    let spacing = level_set.grid_spacing();
    let min = local_bounds.min();
    let max = local_bounds.max();
    let Some((x_start, x_end)) = cell_range(min.x, max.x, origin.x, spacing.x, dimensions[0])
    else {
        return Ok(());
    };
    let Some((y_start, y_end)) = cell_range(min.y, max.y, origin.y, spacing.y, dimensions[1])
    else {
        return Ok(());
    };
    let Some((z_start, z_end)) = cell_range(min.z, max.z, origin.z, spacing.z, dimensions[2])
    else {
        return Ok(());
    };

    for z in z_start..=z_end {
        for y in y_start..=y_end {
            for x in x_start..=x_end {
                let bounds = cell_bounds(level_set, x, y, z)?;
                if !bounds.intersects(&local_bounds) {
                    continue;
                }
                let (minimum, maximum) = corner_value_range(level_set, x, y, z)?;
                if minimum > 0.0 || maximum < 0.0 {
                    continue;
                }
                output
                    .try_reserve(1)
                    .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
                output.push(LevelSetCell {
                    index: cell_index(level_set, x, y, z)?,
                    bounds,
                });
            }
        }
    }
    Ok(())
}

pub(crate) fn query_world_cells(
    level_set: &LevelSet,
    world_transform: &Transform,
    world_bounds: Aabb,
    output: &mut Vec<LevelSetCell>,
) -> Result<(), CollisionError> {
    let local_bounds = super::mesh::local_query_bounds(world_bounds, world_transform)?;
    query_surface_cells(level_set, local_bounds, output)
}

fn locate_axis(
    point: f64,
    origin: f64,
    spacing: f64,
    dimension: usize,
) -> Result<(usize, f64), CollisionError> {
    let coordinate = (point - origin) / spacing;
    if !coordinate.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }
    let last_node = (dimension - 1) as f64;
    let coordinate = coordinate.clamp(0.0, last_node);
    let cell = (coordinate.floor() as usize).min(dimension - 2);
    let fraction = (coordinate - cell as f64).clamp(0.0, 1.0);
    Ok((cell, fraction))
}

fn value_at(level_set: &LevelSet, x: usize, y: usize, z: usize) -> Result<f64, CollisionError> {
    let dimensions = level_set.dimensions();
    let index = z
        .checked_mul(dimensions[1])
        .and_then(|index| index.checked_add(y))
        .and_then(|index| index.checked_mul(dimensions[0]))
        .and_then(|index| index.checked_add(x))
        .ok_or(CollisionError::GeometryBounds)?;
    level_set
        .signed_distances()
        .get(index)
        .copied()
        .ok_or(CollisionError::GeometryBounds)
}

fn cell_range(
    min: f64,
    max: f64,
    origin: f64,
    spacing: f64,
    dimension: usize,
) -> Option<(usize, usize)> {
    let last_node = origin + spacing * (dimension - 1) as f64;
    if max < origin || min > last_node {
        return None;
    }
    let last_cell = dimension.checked_sub(2)?;
    let scaled_min = ((min - origin) / spacing).clamp(0.0, last_cell as f64);
    let scaled_max = ((max - origin) / spacing).clamp(0.0, last_cell as f64);
    if !scaled_min.is_finite() || !scaled_max.is_finite() {
        return None;
    }
    let first = (scaled_min.floor() as usize).saturating_sub(1);
    let last = (scaled_max.floor() as usize).min(last_cell);
    Some((first.min(last), last))
}

fn cell_bounds(level_set: &LevelSet, x: usize, y: usize, z: usize) -> Result<Aabb, CollisionError> {
    let origin = level_set.origin();
    let spacing = level_set.grid_spacing();
    let x0 = origin.x + x as f64 * spacing.x;
    let x1 = origin.x + (x + 1) as f64 * spacing.x;
    let y0 = origin.y + y as f64 * spacing.y;
    let y1 = origin.y + (y + 1) as f64 * spacing.y;
    let z0 = origin.z + z as f64 * spacing.z;
    let z1 = origin.z + (z + 1) as f64 * spacing.z;
    Aabb::new(Vec3::new(x0, y0, z0), Vec3::new(x1, y1, z1))
        .map_err(|_| CollisionError::GeometryBounds)
}

fn corner_value_range(
    level_set: &LevelSet,
    x: usize,
    y: usize,
    z: usize,
) -> Result<(f64, f64), CollisionError> {
    let values = [
        value_at(level_set, x, y, z)?,
        value_at(level_set, x + 1, y, z)?,
        value_at(level_set, x, y + 1, z)?,
        value_at(level_set, x + 1, y + 1, z)?,
        value_at(level_set, x, y, z + 1)?,
        value_at(level_set, x + 1, y, z + 1)?,
        value_at(level_set, x, y + 1, z + 1)?,
        value_at(level_set, x + 1, y + 1, z + 1)?,
    ];
    Ok((
        values.iter().copied().fold(f64::INFINITY, f64::min),
        values.iter().copied().fold(f64::NEG_INFINITY, f64::max),
    ))
}

fn cell_index(level_set: &LevelSet, x: usize, y: usize, z: usize) -> Result<u64, CollisionError> {
    let dimensions = level_set.dimensions();
    let columns = dimensions[0] - 1;
    let rows = dimensions[1] - 1;
    let index = z
        .checked_mul(rows)
        .and_then(|index| index.checked_add(y))
        .and_then(|index| index.checked_mul(columns))
        .and_then(|index| index.checked_add(x))
        .ok_or(CollisionError::ArithmeticOverflow)?;
    u64::try_from(index).map_err(|_| CollisionError::ArithmeticOverflow)
}

fn lerp(a: f64, b: f64, amount: f64) -> Result<f64, CollisionError> {
    let value = a * (1.0 - amount) + b * amount;
    value
        .is_finite()
        .then_some(value)
        .ok_or(CollisionError::ArithmeticOverflow)
}

fn checked_difference(a: f64, b: f64) -> Result<f64, CollisionError> {
    let difference = a - b;
    difference
        .is_finite()
        .then_some(difference)
        .ok_or(CollisionError::ArithmeticOverflow)
}

#[cfg(test)]
#[path = "level_set_tests.rs"]
mod tests;
