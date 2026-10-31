use crate::{aabb::Aabb, shape::HeightField, transform::Transform};
use kapitsa_math::{Vec2, Vec3};

use super::CollisionError;

/// A cell triangle with a stable feature identifier for contact caching.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct HeightFieldTriangle {
    pub(crate) vertices: [Vec3; 3],
    pub(crate) feature_id: u64,
}

/// Appends exact triangles from cells intersected by a local-space query box.
pub(crate) fn query_triangles(
    field: &HeightField,
    local_bounds: Aabb,
    output: &mut Vec<HeightFieldTriangle>,
) -> Result<(), CollisionError> {
    output.clear();
    let spacing = field.grid_spacing();
    let cell_columns = field.columns() - 1;
    let cell_rows = field.rows() - 1;
    let span = Vec2::new(
        spacing.x * cell_columns as f64,
        spacing.y * cell_rows as f64,
    );
    if !span.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }
    let min = local_bounds.min();
    let max = local_bounds.max();
    let Some((first_column, last_column)) =
        cell_range(min.x, max.x, spacing.x, span.x, cell_columns)
    else {
        return Ok(());
    };
    let Some((first_row, last_row)) = cell_range(min.y, max.y, spacing.y, span.y, cell_rows) else {
        return Ok(());
    };

    for row in first_row..=last_row {
        for column in first_column..=last_column {
            let v00 = field_vertex(field, row, column, spacing)?;
            let v10 = field_vertex(field, row, column + 1, spacing)?;
            let v01 = field_vertex(field, row + 1, column, spacing)?;
            let v11 = field_vertex(field, row + 1, column + 1, spacing)?;
            let cell = row
                .checked_mul(cell_columns)
                .and_then(|index| index.checked_add(column))
                .ok_or(CollisionError::ArithmeticOverflow)?;
            append_if_intersects(output, local_bounds, [v00, v10, v11], cell, 0)?;
            append_if_intersects(output, local_bounds, [v00, v11, v01], cell, 1)?;
        }
    }
    Ok(())
}

pub(crate) fn query_world_bounds(
    field: &HeightField,
    world_transform: &Transform,
    world_bounds: Aabb,
    output: &mut Vec<HeightFieldTriangle>,
) -> Result<(), CollisionError> {
    let local_bounds = super::mesh::local_query_bounds(world_bounds, world_transform)?;
    query_triangles(field, local_bounds, output)
}

fn cell_range(
    min: f64,
    max: f64,
    spacing: f64,
    span: f64,
    cell_count: usize,
) -> Option<(usize, usize)> {
    if max < 0.0 || min > span {
        return None;
    }
    let last_cell = cell_count.checked_sub(1)?;
    let last_cell_value = last_cell as f64;
    let scaled_min = (min / spacing).clamp(0.0, last_cell_value);
    let scaled_max = (max / spacing).clamp(0.0, last_cell_value);
    if !scaled_min.is_finite() || !scaled_max.is_finite() {
        return None;
    }
    let first = (scaled_min.floor() as usize).saturating_sub(1);
    let last = (scaled_max.floor() as usize).min(last_cell);
    Some((first.min(last), last))
}

fn field_vertex(
    field: &HeightField,
    row: usize,
    column: usize,
    spacing: Vec2,
) -> Result<Vec3, CollisionError> {
    let index = row
        .checked_mul(field.columns())
        .and_then(|index| index.checked_add(column))
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let height = *field
        .heights()
        .get(index)
        .ok_or(CollisionError::GeometryBounds)?;
    let vertex = Vec3::new(column as f64 * spacing.x, row as f64 * spacing.y, height);
    vertex
        .is_finite()
        .then_some(vertex)
        .ok_or(CollisionError::ArithmeticOverflow)
}

fn append_if_intersects(
    output: &mut Vec<HeightFieldTriangle>,
    bounds: Aabb,
    vertices: [Vec3; 3],
    cell: usize,
    triangle: u64,
) -> Result<(), CollisionError> {
    let min = vertices[0].min(vertices[1]).min(vertices[2]);
    let max = vertices[0].max(vertices[1]).max(vertices[2]);
    let triangle_bounds = Aabb::new(min, max).map_err(|_| CollisionError::GeometryBounds)?;
    if !triangle_bounds.intersects(&bounds) {
        return Ok(());
    }
    let cell = u64::try_from(cell).map_err(|_| CollisionError::ArithmeticOverflow)?;
    let feature_id = cell
        .checked_mul(2)
        .and_then(|feature| feature.checked_add(triangle))
        .ok_or(CollisionError::ArithmeticOverflow)?;
    output
        .try_reserve(1)
        .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
    output.push(HeightFieldTriangle {
        vertices,
        feature_id,
    });
    Ok(())
}

#[cfg(test)]
#[path = "height_field_tests.rs"]
mod tests;
