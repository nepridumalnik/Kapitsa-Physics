//! Scalar conservative voxel geometry, independent of forces and center of mass.
//!
//! Cells intersecting a triangle are occupied in full. Thin surfaces survive, but
//! volume and surface area are resolution dependent. Voxel normals are axis-aligned:
//! they are not mesh normals and must not be used as a convergent pressure model.
//! Closed instances use solid flood-fill semantics (sealed cavities are filled).
//! Indexed seams must be welded; self-intersections are not diagnosed.

use std::fmt;
mod visibility;

use kapitsa_math::Vec3;

use crate::{shape::TriangleMesh, transform::Transform, world::PartId};

/// Binding-independent owner index within a cooked shape. Slots are dense
/// indices fixed at cook time; a binding resolves them to concrete parts.
/// Slot order defines overlap tie-breaks for shared prepared geometry.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct OwnerSlot(u32);

impl OwnerSlot {
    /// Creates a slot from its dense index. Resolution bounds are checked by
    /// the binding, not by the slot itself.
    pub fn new(index: u32) -> Self {
        Self(index)
    }

    /// Dense index of this slot within its cooked shape.
    pub fn index(self) -> u32 {
        self.0
    }
}

/// Attribution within one body's local frame. Lowest owner wins overlap ties.
///
/// Cooked shared shapes use [`GeometryOwner::Slot`] owners so prepared
/// geometry stays independent of any vessel's part identities; body-bound
/// evaluation uses [`GeometryOwner::Part`] after the binding resolves slots to
/// parts. Do not mix `Part` and `Slot` owners within one preparation.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum GeometryOwner {
    Body,
    Part(PartId),
    Slot(OwnerSlot),
}

/// Borrowed geometry. Preparation neither clones nor takes ownership of the mesh.
#[derive(Clone, Copy)]
pub struct GeometryInstance<'a> {
    pub mesh: &'a TriangleMesh,
    pub local_transform: Transform,
    pub owner: GeometryOwner,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OpenMeshPolicy {
    Reject,
    SurfaceOnly,
}

/// Hard ceilings also apply: 1,048,576 cells and 100,000,000 work units.
/// Work counts triangle/box tests and per-instance cell scans.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GeometryConfig {
    pub cell_size: f64,
    pub max_cells: usize,
    pub max_work: usize,
    pub open_mesh_policy: OpenMeshPolicy,
    pub weld_tolerance: Option<f64>,
    pub max_patch_work: usize,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GeometryError {
    InvalidConfig,
    EmptyGeometry,
    InvalidGeometry,
    OpenMesh,
    CellBudgetExceeded,
    WorkBudgetExceeded,
    AllocationFailed,
    ArithmeticOverflow,
    PrecisionLoss,
}

impl fmt::Display for GeometryError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "aerodynamic geometry preparation failed: {self:?}")
    }
}
impl std::error::Error for GeometryError {}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GeometrySection {
    pub area: f64,
    pub centroid: Option<Vec3>,
}

/// Exposed voxel face, not an original triangle or an aerodynamic force element.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GeometryFace {
    pub center: Vec3,
    pub normal: Vec3,
    pub area: f64,
    pub owner: GeometryOwner,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SurfaceTriangle {
    pub vertices: [Vec3; 3],
    pub normal: Vec3,
    pub area: f64,
    pub owner: GeometryOwner,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PreparedSurfacePatch {
    pub vertices: [Vec3; 3],
    pub center: Vec3,
    pub normal: Vec3,
    pub area: f64,
    pub owner: GeometryOwner,
}

#[derive(Clone, Debug, PartialEq)]
pub struct PreparedSurfacePatches {
    patches: Vec<PreparedSurfacePatch>,
}

impl PreparedSurfacePatches {
    pub fn as_slice(&self) -> &[PreparedSurfacePatch] {
        &self.patches
    }
    pub fn iter(&self) -> impl Iterator<Item = &PreparedSurfacePatch> {
        self.patches.iter()
    }
    pub fn len(&self) -> usize {
        self.patches.len()
    }
    pub fn is_empty(&self) -> bool {
        self.patches.is_empty()
    }
    pub(crate) fn retain(&mut self, keep: impl FnMut(&PreparedSurfacePatch) -> bool) {
        self.patches.retain(keep);
    }
}

/// Immutable union geometry. World motion and COM changes do not affect it.
#[derive(Debug, PartialEq)]
pub struct PreparedGeometry {
    origin: Vec3,
    cell_size: f64,
    dimensions: [usize; 3],
    occupancy: Vec<Option<GeometryOwner>>,
    sections: [Vec<GeometrySection>; 3],
    volume: f64,
}

fn allocated<T: Clone>(len: usize, value: T) -> Result<Vec<T>, crate::Error> {
    let mut values = Vec::new();
    values
        .try_reserve_exact(len)
        .map_err(|_| crate::Error::from(GeometryError::AllocationFailed))?;
    values.resize(len, value);
    Ok(values)
}

fn checked_push<T>(values: &mut Vec<T>, value: T) -> Result<(), crate::Error> {
    values
        .try_reserve(1)
        .map_err(|_| crate::Error::from(GeometryError::AllocationFailed))?;
    values.push(value);
    Ok(())
}

fn surface_triangle_count(instances: &[GeometryInstance<'_>]) -> Result<usize, crate::Error> {
    instances.iter().try_fold(0usize, |total, instance| {
        total
            .checked_add(instance.mesh.triangles().len())
            .ok_or_else(|| crate::Error::from(GeometryError::WorkBudgetExceeded))
    })
}

const MAX_SURFACE_WORK: usize = 100_000_000;

fn check_surface_work(work: Option<usize>, max_work: usize) -> Result<(), crate::Error> {
    if max_work == 0 || work.is_none_or(|w| w > max_work.min(MAX_SURFACE_WORK)) {
        return Err(crate::Error::from(GeometryError::WorkBudgetExceeded));
    }
    Ok(())
}

// Up-front accounting reservation; budgeted_sort additionally charges actual
// comparisons and swaps as they occur.
fn sort_work_estimate(len: usize) -> Option<usize> {
    let factor = usize::BITS as usize - len.leading_zeros() as usize + 1;
    len.checked_mul(factor)
}

// Includes surface_triangles' four units per triangle; this is a reservation,
// not an additional allowance for the same preparation work.
const TRIANGLE_PREPARATION_WORK: usize = 16;
const SURFACE_TRIANGLE_WORK: usize = 4;

fn grid_build_work_estimate(entries: usize, cells: usize, scans: usize) -> Option<usize> {
    // Initialize both buffers and scan counts to construct prefix offsets.
    const CELL_PASSES: usize = 3;
    entries
        .checked_mul(3)? // Count, initialize, and populate entries.
        .checked_add(cells.checked_mul(CELL_PASSES)?)?
        .checked_add(1)?
        .checked_add(scans)
}

fn charge_surface_work(
    work: &mut usize,
    amount: usize,
    max_work: usize,
) -> Result<(), crate::Error> {
    let next = work.checked_add(amount);
    check_surface_work(next, max_work)?;
    *work = next.ok_or_else(|| crate::Error::from(GeometryError::WorkBudgetExceeded))?;
    Ok(())
}

fn allocate_candidate_marks(
    count: usize,
    work: &mut usize,
    max_work: usize,
) -> Result<Vec<u32>, crate::Error> {
    charge_surface_work(work, count, max_work)?;
    allocated(count, 0u32)
}

/// In-place heapsort with a checked charge before each comparison and swap.
fn budgeted_sort<T>(
    values: &mut [T],
    work: &mut usize,
    max_work: usize,
    compare: impl Fn(&T, &T) -> std::cmp::Ordering,
) -> Result<(), crate::Error> {
    fn sift<T>(
        values: &mut [T],
        mut root: usize,
        end: usize,
        work: &mut usize,
        max_work: usize,
        compare: &impl Fn(&T, &T) -> std::cmp::Ordering,
    ) -> Result<(), crate::Error> {
        while root < end / 2 {
            let mut child = root * 2 + 1;
            if child + 1 < end {
                charge_surface_work(work, 1, max_work)?;
                if compare(&values[child], &values[child + 1]).is_lt() {
                    child += 1;
                }
            }
            charge_surface_work(work, 1, max_work)?;
            if !compare(&values[root], &values[child]).is_lt() {
                break;
            }
            charge_surface_work(work, 1, max_work)?;
            values.swap(root, child);
            root = child;
        }
        Ok(())
    }

    let len = values.len();
    for root in (0..len / 2).rev() {
        sift(values, root, len, work, max_work, &compare)?;
    }
    for end in (1..len).rev() {
        charge_surface_work(work, 1, max_work)?;
        values.swap(0, end);
        sift(values, 0, end, work, max_work, &compare)?;
    }
    Ok(())
}

/// Temporary broad phase for visibility blockers. Each query is an axis-aligned
/// conservative bound of the source triangle swept to the prepared geometry's
/// bounds along the ray; it may return extra triangles, but never omits a hit.
struct TriangleGrid {
    origin: Vec3,
    cell_size: Vec3,
    dimensions: [usize; 3],
    offsets: Vec<usize>,
    triangles: Vec<usize>,
    build_work: usize,
}

impl TriangleGrid {
    fn build(triangles: &[SurfaceTriangle], max_work: usize) -> Result<Self, crate::Error> {
        if triangles.is_empty() {
            return Err(crate::Error::from(GeometryError::EmptyGeometry));
        }
        // Bounds, preflight, bucket counting and bucket population scan the input.
        const TRIANGLE_SCAN_WORK: usize = 4;
        let scan_work = triangles
            .len()
            .checked_mul(TRIANGLE_SCAN_WORK)
            .ok_or_else(|| crate::Error::from(GeometryError::WorkBudgetExceeded))?;
        check_surface_work(Some(scan_work), max_work)?;
        let mut min = Vec3::splat(f64::INFINITY);
        let mut max = Vec3::splat(f64::NEG_INFINITY);
        for triangle in triangles {
            for &point in &triangle.vertices {
                if !point.is_finite() {
                    return Err(crate::Error::from(GeometryError::ArithmeticOverflow));
                }
                min = min.min(point);
                max = max.max(point);
            }
        }
        let extent = max - min;
        let target = (triangles.len() as f64).cbrt().ceil().clamp(1.0, 16.0) as usize;
        let dimensions = components(extent).map(|value| if value == 0.0 { 1 } else { target });
        let cell_size = vector(components(extent).map(|value| {
            if value == 0.0 {
                1.0
            } else {
                value / target as f64
            }
        }));
        let cell_count = dimensions
            .into_iter()
            .try_fold(1usize, |n, d| n.checked_mul(d))
            .ok_or_else(|| crate::Error::from(GeometryError::WorkBudgetExceeded))?;
        let cell = |point: Vec3| -> Result<[usize; 3], crate::Error> {
            let values = components((point - min) / cell_size);
            if !values.iter().all(|value| value.is_finite()) {
                return Err(crate::Error::from(GeometryError::PrecisionLoss));
            }
            Ok(values.map(|value| (value.floor().max(0.0) as usize).min(target.saturating_sub(1))))
        };
        let triangle_bounds = |triangle: &SurfaceTriangle| -> Result<_, crate::Error> {
            let lower = cell(
                triangle
                    .vertices
                    .iter()
                    .fold(Vec3::splat(f64::INFINITY), |v, &p| v.min(p)),
            )?;
            let upper = cell(
                triangle
                    .vertices
                    .iter()
                    .fold(Vec3::splat(f64::NEG_INFINITY), |v, &p| v.max(p)),
            )?;
            Ok((lower, upper))
        };
        let mut entry_count = 0usize;
        for triangle in triangles {
            let (lower, upper) = triangle_bounds(triangle)?;
            let spans = [
                upper[0].saturating_sub(lower[0]).saturating_add(1),
                upper[1].saturating_sub(lower[1]).saturating_add(1),
                upper[2].saturating_sub(lower[2]).saturating_add(1),
            ];
            entry_count = entry_count
                .checked_add(
                    spans[0]
                        .checked_mul(spans[1])
                        .and_then(|n| n.checked_mul(spans[2]))
                        .ok_or_else(|| crate::Error::from(GeometryError::WorkBudgetExceeded))?,
                )
                .ok_or_else(|| crate::Error::from(GeometryError::WorkBudgetExceeded))?;
            // The estimate is monotone in entry_count: once it exceeds the
            // budget, later triangles cannot make this construction viable.
            let partial_work = grid_build_work_estimate(entry_count, cell_count, scan_work);
            check_surface_work(partial_work, max_work)?;
        }
        let build_work = grid_build_work_estimate(entry_count, cell_count, scan_work)
            .ok_or_else(|| crate::Error::from(GeometryError::WorkBudgetExceeded))?;
        check_surface_work(Some(build_work), max_work)?;
        let mut cursors = allocated(cell_count, 0usize)?;
        let occupied_cells = |triangle: &SurfaceTriangle| -> Result<_, crate::Error> {
            let (lower, upper) = triangle_bounds(triangle)?;
            Ok((lower[2]..=upper[2]).flat_map(move |z| {
                (lower[1]..=upper[1]).flat_map(move |y| {
                    (lower[0]..=upper[0]).map(move |x| index([x, y, z], dimensions))
                })
            }))
        };
        for triangle in triangles {
            for cell in occupied_cells(triangle)? {
                cursors[cell] += 1;
            }
        }
        let mut offsets = allocated(cell_count + 1, 0usize)?;
        for (cell, cursor) in cursors.iter_mut().enumerate() {
            // Preflight bounds the sum of all bucket counts.
            offsets[cell + 1] = offsets[cell] + *cursor;
            *cursor = offsets[cell];
        }
        let mut indexed_triangles = allocated(entry_count, 0usize)?;
        // Insertion in input order preserves the canonical order within a bucket.
        for (triangle_index, triangle) in triangles.iter().enumerate() {
            for cell in occupied_cells(triangle)? {
                indexed_triangles[cursors[cell]] = triangle_index;
                cursors[cell] += 1;
            }
        }
        Ok(Self {
            origin: min,
            cell_size,
            dimensions,
            offsets,
            triangles: indexed_triangles,
            build_work,
        })
    }

    fn candidates(
        &self,
        source: &SurfaceTriangle,
        direction: Vec3,
        marks: &mut [u32],
        stamp: u32,
        max_work: usize,
    ) -> Result<(Vec<usize>, usize), crate::Error> {
        let source_min = source
            .vertices
            .iter()
            .fold(Vec3::splat(f64::INFINITY), |v, &p| v.min(p));
        let source_max = source
            .vertices
            .iter()
            .fold(Vec3::splat(f64::NEG_INFINITY), |v, &p| v.max(p));
        let global_max = self.origin + self.cell_size * vector(self.dimensions.map(|d| d as f64));
        let lower =
            components(source_min).map(|value| if value.is_finite() { value } else { f64::NAN });
        let upper =
            components(source_max).map(|value| if value.is_finite() { value } else { f64::NAN });
        let mut swept_min = lower;
        let mut swept_max = upper;
        let direction = components(direction);
        let global_max = components(global_max);
        let origin = components(self.origin);
        for axis in 0..3 {
            if direction[axis] > 0.0 {
                swept_max[axis] = global_max[axis];
            }
            if direction[axis] < 0.0 {
                swept_min[axis] = origin[axis];
            }
        }
        let cell_size = components(self.cell_size);
        let to_cell = |point: [f64; 3]| -> Result<[usize; 3], crate::Error> {
            if !point.iter().all(|value| value.is_finite()) {
                return Err(crate::Error::from(GeometryError::PrecisionLoss));
            }
            Ok(std::array::from_fn(|axis| {
                ((point[axis] - origin[axis]) / cell_size[axis])
                    .floor()
                    .max(0.0) as usize
            }))
        };
        let mut lower = to_cell(swept_min)?;
        let mut upper = to_cell(swept_max)?;
        for axis in 0..3 {
            lower[axis] = lower[axis].min(self.dimensions[axis].saturating_sub(1));
            upper[axis] = upper[axis].min(self.dimensions[axis].saturating_sub(1));
        }
        let mut result = Vec::new();
        let mut query_work = 0usize;
        for z in lower[2]..=upper[2] {
            for y in lower[1]..=upper[1] {
                for x in lower[0]..=upper[0] {
                    query_work = query_work
                        .checked_add(1)
                        .ok_or_else(|| crate::Error::from(GeometryError::WorkBudgetExceeded))?;
                    check_surface_work(Some(query_work), max_work)?;
                    let range = index([x, y, z], self.dimensions);
                    for &triangle in &self.triangles[self.offsets[range]..self.offsets[range + 1]] {
                        query_work = query_work
                            .checked_add(1)
                            .ok_or_else(|| crate::Error::from(GeometryError::WorkBudgetExceeded))?;
                        check_surface_work(Some(query_work), max_work)?;
                        if marks[triangle] != stamp {
                            checked_push(&mut result, triangle)?;
                            marks[triangle] = stamp;
                        }
                    }
                }
            }
        }
        Ok((result, query_work))
    }
}

fn compare_vertex(a: &Vec3, b: &Vec3) -> std::cmp::Ordering {
    // Normalize signed zero so equivalent coordinates have the same ordering.
    let canonical = |v: f64| if v == 0.0 { 0.0 } else { v };
    canonical(a.x)
        .total_cmp(&canonical(b.x))
        .then_with(|| canonical(a.y).total_cmp(&canonical(b.y)))
        .then_with(|| canonical(a.z).total_cmp(&canonical(b.z)))
}

/// Two-sided Möller–Trumbore intersection, inclusive of triangle edges.
/// No geometric epsilon: t > 0 preserves arbitrarily close genuine blockers.
#[cfg(test)]
fn ray_hits_triangle(
    origin: Vec3,
    direction: Vec3,
    vertices: [Vec3; 3],
) -> Result<bool, crate::Error> {
    let [a, b, c] = vertices;
    let e1 = b - a;
    let e2 = c - a;
    let p = direction.cross(e2);
    let determinant = e1.dot(p);
    if !determinant.is_finite() {
        return Err(crate::Error::from(GeometryError::ArithmeticOverflow));
    }
    if determinant == 0.0 {
        return Ok(false);
    }
    let offset = origin - a;
    let q = offset.cross(e1);
    let u = offset.dot(p) / determinant;
    let v = direction.dot(q) / determinant;
    let t = e2.dot(q) / determinant;
    if ![u, v, t].iter().all(|n| n.is_finite()) {
        return Err(crate::Error::from(GeometryError::ArithmeticOverflow));
    }
    Ok(t > 0.0 && u >= 0.0 && v >= 0.0 && u + v <= 1.0)
}

fn components(v: Vec3) -> [f64; 3] {
    [v.x, v.y, v.z]
}
fn vector(v: [f64; 3]) -> Vec3 {
    Vec3::new(v[0], v[1], v[2])
}
fn index(p: [usize; 3], d: [usize; 3]) -> usize {
    (p[2] * d[1] + p[1]) * d[0] + p[0]
}
fn coordinates(i: usize, d: [usize; 3]) -> [usize; 3] {
    [i % d[0], (i / d[0]) % d[1], i / (d[0] * d[1])]
}
fn neighbor(mut p: [usize; 3], axis: usize, positive: bool, d: [usize; 3]) -> Option<usize> {
    p[axis] = if positive {
        p[axis].checked_add(1)?
    } else {
        p[axis].checked_sub(1)?
    };
    (p[axis] < d[axis]).then(|| index(p, d))
}

fn closed(mesh: &TriangleMesh, weld_tolerance: Option<f64>) -> Result<bool, crate::Error> {
    let welded = if let Some(tolerance) = weld_tolerance {
        if !tolerance.is_finite() || tolerance <= 0.0 {
            return Err(crate::Error::from(GeometryError::InvalidConfig));
        }
        let mut representatives: Vec<usize> = Vec::new();
        representatives
            .try_reserve_exact(mesh.vertices().len())
            .map_err(|_| crate::Error::from(GeometryError::AllocationFailed))?;
        for vertex in mesh.vertices() {
            let representative = representatives.iter().position(|&index| {
                (mesh.vertices()[index] - *vertex).length_squared() <= tolerance * tolerance
            });
            let index = match representative {
                Some(index) => index,
                None => representatives.len(),
            };
            representatives.push(index);
        }
        Some(representatives)
    } else {
        None
    };
    let mut edges = Vec::new();
    edges
        .try_reserve_exact(
            mesh.triangles()
                .len()
                .checked_mul(3)
                .ok_or_else(|| crate::Error::from(GeometryError::WorkBudgetExceeded))?,
        )
        .map_err(|_| crate::Error::from(GeometryError::AllocationFailed))?;
    for &[a, b, c] in mesh.triangles() {
        for (a, b) in [(a, b), (b, c), (c, a)] {
            let a = welded.as_ref().map_or(a, |w| w[a]);
            let b = welded.as_ref().map_or(b, |w| w[b]);
            edges.push((a.min(b), a.max(b), a < b));
        }
    }
    edges.sort_unstable();
    let (pairs, remainder) = edges.as_chunks::<2>();
    let manifold = remainder.is_empty()
        && pairs
            .iter()
            .all(|e| e[0].0 == e[1].0 && e[0].1 == e[1].1 && e[0].2 != e[1].2)
        && !edges
            .windows(3)
            .any(|e| e[0].0 == e[2].0 && e[0].1 == e[2].1);
    if !manifold {
        return Ok(false);
    }
    let reference = mesh.vertices()[mesh.triangles()[0][0]];
    let scale = mesh
        .triangles()
        .iter()
        .flat_map(|triangle| triangle.iter())
        .map(|&index| (mesh.vertices()[index] - reference).abs().max_element())
        .fold(0.0_f64, f64::max);
    if !scale.is_finite() || scale == 0.0 {
        return Err(crate::Error::from(GeometryError::InvalidGeometry));
    }
    let mut volume = 0.0;
    for &[a, b, c] in mesh.triangles() {
        let [a, b, c] = [a, b, c].map(|i| (mesh.vertices()[i] - reference) / scale);
        volume += a.dot(b.cross(c));
    }
    Ok(volume.is_finite() && volume != 0.0)
}

/// Separating-axis triangle/box test: box axes, triangle normal, nine edge axes.
fn intersects(triangle: [Vec3; 3], center: Vec3, half: f64) -> Result<bool, crate::Error> {
    let v = triangle.map(|v| v - center);
    let edges = [v[1] - v[0], v[2] - v[1], v[0] - v[2]];
    let axes = [Vec3::X, Vec3::Y, Vec3::Z];
    let normal = edges[0].cross(edges[1]);
    for axis in axes
        .into_iter()
        .chain([normal])
        .chain(edges.into_iter().flat_map(|e| axes.map(|a| e.cross(a))))
    {
        let scale = axis.abs().max_element();
        if !scale.is_finite() {
            return Err(crate::Error::from(GeometryError::ArithmeticOverflow));
        }
        if scale == 0.0 {
            continue;
        }
        let axis = axis / scale;
        let projected = v.map(|p| p.dot(axis));
        let radius = half * (axis.x.abs() + axis.y.abs() + axis.z.abs());
        if !radius.is_finite() || !projected.iter().all(|v| v.is_finite()) {
            return Err(crate::Error::from(GeometryError::ArithmeticOverflow));
        }
        // Include boundary contact so grid-aligned zero-thickness surfaces survive.
        if projected.iter().all(|&v| v > radius) || projected.iter().all(|&v| v < -radius) {
            return Ok(false);
        }
    }
    Ok(true)
}

// Bounds used both by the work preflight and the rasterization itself.
fn triangle_grid_bounds(
    triangle: [Vec3; 3],
    origin: Vec3,
    h: f64,
    dimensions: [usize; 3],
) -> ([usize; 3], [usize; 3]) {
    let triangle_min = triangle
        .iter()
        .fold(Vec3::splat(f64::INFINITY), |min, &v| min.min(v));
    let triangle_max = triangle
        .iter()
        .fold(Vec3::splat(f64::NEG_INFINITY), |max, &v| max.max(v));
    let lower = components((triangle_min - origin) / h).map(|v| v.floor().max(0.0) as usize);
    let upper = components((triangle_max - origin) / h).map(|v| v.ceil().max(0.0) as usize);
    (
        std::array::from_fn(|axis| lower[axis].min(dimensions[axis].saturating_sub(1))),
        std::array::from_fn(|axis| upper[axis].min(dimensions[axis].saturating_sub(1))),
    )
}

fn flood_voxels(
    shell: &[bool],
    visited: &mut [bool],
    dimensions: [usize; 3],
    queue: &mut Vec<usize>,
) {
    let mut head = 0;
    while head < queue.len() {
        let p = coordinates(queue[head], dimensions);
        head += 1;
        for axis in 0..3 {
            for positive in [false, true] {
                if let Some(i) = neighbor(p, axis, positive, dimensions)
                    && !shell[i]
                    && !visited[i]
                {
                    visited[i] = true;
                    queue.push(i);
                }
            }
        }
    }
}

fn has_nonzero_winding(instance: &GeometryInstance<'_>, point: Vec3) -> Result<bool, crate::Error> {
    // Jacobson et al. (2013), section 4.1, equation (2), evaluated with unit rays.
    // Away from an oriented closed surface the sum is an integer number of turns.
    const MAX_WINDING_ERROR: f64 = 0.125;
    let unit_ray = |index: usize| -> Result<Vec3, crate::Error> {
        let vertex = instance
            .local_transform
            .transform_point(instance.mesh.vertices()[index]);
        let ray = vertex - point;
        let length = ray
            .checked_length()
            .filter(|length| *length > 0.0)
            .ok_or(GeometryError::PrecisionLoss)?;
        let unit = ray / length;
        if !unit.is_finite() {
            return Err(GeometryError::PrecisionLoss.into());
        }
        Ok(unit)
    };
    let mut solid_angle = 0.0;
    for &[a, b, c] in instance.mesh.triangles() {
        let (a, b, c) = (unit_ray(a)?, unit_ray(b)?, unit_ray(c)?);
        let numerator = a.dot(b.cross(c));
        let denominator = 1.0 + a.dot(b) + b.dot(c) + c.dot(a);
        if numerator == 0.0 && denominator <= 0.0 {
            return Err(GeometryError::PrecisionLoss.into());
        }
        solid_angle += 2.0 * numerator.atan2(denominator);
    }
    let winding = solid_angle / (4.0 * std::f64::consts::PI);
    let integral = winding.round();
    if !winding.is_finite() || (winding - integral).abs() > MAX_WINDING_ERROR {
        return Err(GeometryError::PrecisionLoss.into());
    }
    Ok(integral != 0.0)
}

impl PreparedGeometry {
    pub fn prepare(
        instances: &[GeometryInstance<'_>],
        config: GeometryConfig,
    ) -> Result<Self, crate::Error> {
        Self::prepare_inner(instances, config, false)
    }

    /// Preserves resolved cavities in closed oriented meshes. The walls of a
    /// cavity must have winding opposite to the enclosing shell in the same
    /// instance. Cells touched by triangles remain conservatively occupied.
    /// The region traversal and solid-angle evaluations count toward `config.max_work`.
    pub fn prepare_with_oriented_cavities(
        instances: &[GeometryInstance<'_>],
        config: GeometryConfig,
    ) -> Result<Self, crate::Error> {
        Self::prepare_inner(instances, config, true)
    }

    fn prepare_inner(
        instances: &[GeometryInstance<'_>],
        config: GeometryConfig,
        oriented_cavities: bool,
    ) -> Result<Self, crate::Error> {
        let h = config.cell_size;
        if !h.is_finite()
            || h <= 0.0
            || !(h * h * h).is_finite()
            || h * h * h == 0.0
            || config.max_cells == 0
            || config.max_cells > 1_048_576
            || config.max_work == 0
            || config.max_work > 100_000_000
        {
            return Err(crate::Error::from(GeometryError::InvalidConfig));
        }
        if instances.is_empty() {
            return Err(crate::Error::from(GeometryError::EmptyGeometry));
        }
        let triangle_count = instances
            .iter()
            .try_fold(0usize, |n, i| n.checked_add(i.mesh.triangles().len()))
            .ok_or_else(|| crate::Error::from(GeometryError::WorkBudgetExceeded))?;
        let vertex_count = instances
            .iter()
            .try_fold(0usize, |n, i| n.checked_add(i.mesh.vertices().len()))
            .ok_or_else(|| crate::Error::from(GeometryError::WorkBudgetExceeded))?;
        if triangle_count
            .checked_add(vertex_count)
            .is_none_or(|n| n > config.max_work)
        {
            return Err(crate::Error::from(GeometryError::WorkBudgetExceeded));
        }
        // Closure checks sort three edges per triangle; optional positional
        // welding compares each vertex with preceding representatives.
        let preprocessing_work = instances
            .iter()
            .try_fold(0usize, |work, instance| {
                let edges = instance.mesh.triangles().len().checked_mul(3)?;
                let sorting =
                    edges.checked_mul(usize::BITS as usize - edges.leading_zeros() as usize + 1)?;
                let welding = if config.weld_tolerance.is_some() {
                    instance
                        .mesh
                        .vertices()
                        .len()
                        .checked_mul(instance.mesh.vertices().len())?
                } else {
                    instance.mesh.vertices().len()
                };
                work.checked_add(sorting)?.checked_add(welding)
            })
            .ok_or_else(|| crate::Error::from(GeometryError::WorkBudgetExceeded))?;
        if preprocessing_work > config.max_work {
            return Err(crate::Error::from(GeometryError::WorkBudgetExceeded));
        }
        let mut min = Vec3::splat(f64::INFINITY);
        let mut max = Vec3::splat(f64::NEG_INFINITY);
        for instance in instances {
            for triangle in instance.mesh.triangles() {
                for &index in triangle {
                    let vertex = instance.mesh.vertices()[index];
                    let v = instance.local_transform.transform_point(vertex);
                    if !v.is_finite() {
                        return Err(crate::Error::from(GeometryError::ArithmeticOverflow));
                    }
                    min = min.min(v);
                    max = max.max(v);
                }
            }
        }
        let origin = min - Vec3::splat(h);
        let extents = components((max - min) / h);
        if !origin.is_finite()
            || !extents
                .iter()
                .all(|x| x.is_finite() && *x >= 0.0 && *x <= config.max_cells as f64)
        {
            return Err(crate::Error::from(GeometryError::CellBudgetExceeded));
        }
        if (origin + Vec3::splat(h * 0.5)).x == origin.x
            || (origin + Vec3::splat(h * 0.5)).y == origin.y
            || (origin + Vec3::splat(h * 0.5)).z == origin.z
        {
            return Err(crate::Error::from(GeometryError::InvalidGeometry));
        }
        let dimensions = extents.map(|x| (x.ceil() as usize).max(1) + 2);
        let count = dimensions
            .into_iter()
            .try_fold(1usize, |n, d| n.checked_mul(d))
            .ok_or_else(|| crate::Error::from(GeometryError::CellBudgetExceeded))?;
        if count > config.max_cells {
            return Err(crate::Error::from(GeometryError::CellBudgetExceeded));
        }
        // Charge full-grid passes and the actual conservative triangle bounds.
        // This pass happens before grid allocation and uses the same bounds as
        // rasterization, so a rejected request cannot allocate a partial grid.
        let mut work = count
            .checked_mul(instances.len().saturating_mul(10).saturating_add(7))
            .and_then(|n| n.checked_add(vertex_count))
            .and_then(|n| n.checked_add(preprocessing_work))
            .and_then(|n| n.checked_add(triangle_count.saturating_mul(10)))
            .ok_or_else(|| crate::Error::from(GeometryError::WorkBudgetExceeded))?;
        if oriented_cavities {
            // At most one additional visit, seed check, six neighbor checks
            // and one occupancy write per cell across enclosed regions.
            const REGION_WORK_PER_CELL: usize = 9;
            work = count
                .checked_mul(instances.len())
                .and_then(|n| n.checked_mul(REGION_WORK_PER_CELL))
                .and_then(|n| work.checked_add(n))
                .ok_or(GeometryError::WorkBudgetExceeded)?;
        }
        for instance in instances {
            for &[a, b, c] in instance.mesh.triangles() {
                let triangle = [a, b, c].map(|i| {
                    instance
                        .local_transform
                        .transform_point(instance.mesh.vertices()[i])
                });
                let (lower, upper) = triangle_grid_bounds(triangle, origin, h, dimensions);
                let cells = (0..3)
                    .try_fold(1_usize, |n, axis| {
                        upper[axis]
                            .checked_sub(lower[axis])?
                            .checked_add(1)
                            .and_then(|width| n.checked_mul(width))
                    })
                    .ok_or_else(|| crate::Error::from(GeometryError::WorkBudgetExceeded))?;
                work = work
                    .checked_add(cells)
                    .ok_or_else(|| crate::Error::from(GeometryError::WorkBudgetExceeded))?;
                if work > config.max_work {
                    return Err(crate::Error::from(GeometryError::WorkBudgetExceeded));
                }
            }
        }
        if work > config.max_work {
            return Err(crate::Error::from(GeometryError::WorkBudgetExceeded));
        }
        let upper = origin + vector(dimensions.map(|d| d as f64 * h));
        if !upper.is_finite() {
            return Err(crate::Error::from(GeometryError::ArithmeticOverflow));
        }
        if !(h * h * count as f64 * 6.0).is_finite() || !(h * h * h * count as f64).is_finite() {
            return Err(crate::Error::from(GeometryError::ArithmeticOverflow));
        }
        let mut occupancy: Vec<Option<GeometryOwner>> = allocated(count, None)?;
        let mut shell = allocated(count, false)?;
        let mut exterior = allocated(count, false)?;
        let mut queue = Vec::new();
        queue
            .try_reserve_exact(count)
            .map_err(|_| crate::Error::from(GeometryError::AllocationFailed))?;
        for instance in instances {
            let is_closed = closed(instance.mesh, config.weld_tolerance)?;
            if !is_closed && config.open_mesh_policy == OpenMeshPolicy::Reject {
                return Err(crate::Error::from(GeometryError::OpenMesh));
            }
            shell.fill(false);
            exterior.fill(false);
            queue.clear();
            for &[a, b, c] in instance.mesh.triangles() {
                let triangle = [a, b, c].map(|i| {
                    instance
                        .local_transform
                        .transform_point(instance.mesh.vertices()[i])
                });
                let (lower, upper) = triangle_grid_bounds(triangle, origin, h, dimensions);
                for z in lower[2]..=upper[2] {
                    for y in lower[1]..=upper[1] {
                        for x in lower[0]..=upper[0] {
                            let i = index([x, y, z], dimensions);
                            let center = origin
                                + vector([
                                    (x as f64 + 0.5) * h,
                                    (y as f64 + 0.5) * h,
                                    (z as f64 + 0.5) * h,
                                ]);
                            if intersects(triangle, center, h * 0.5)? {
                                shell[i] = true;
                            }
                        }
                    }
                }
            }
            if is_closed {
                for i in 0..count {
                    let p = coordinates(i, dimensions);
                    if (0..3).any(|a| p[a] == 0 || p[a] + 1 == dimensions[a]) && !shell[i] {
                        exterior[i] = true;
                        queue.push(i);
                    }
                }
                flood_voxels(&shell, &mut exterior, dimensions, &mut queue);
                if oriented_cavities {
                    for i in 0..count {
                        if exterior[i] || shell[i] {
                            continue;
                        }
                        work = work
                            .checked_add(instance.mesh.triangles().len())
                            .filter(|&next| next <= config.max_work)
                            .ok_or(GeometryError::WorkBudgetExceeded)?;
                        // Adjacent non-surface cells share a path that cannot
                        // cross a triangle, so winding is invariant in the region.
                        let p = coordinates(i, dimensions);
                        let center = origin + vector(p.map(|v| (v as f64 + 0.5) * h));
                        let filled = has_nonzero_winding(instance, center)?;
                        queue.clear();
                        exterior[i] = true;
                        queue.push(i);
                        flood_voxels(&shell, &mut exterior, dimensions, &mut queue);
                        if filled {
                            for &cell in &queue {
                                shell[cell] = true;
                            }
                        }
                    }
                }
            }
            for i in 0..count {
                if shell[i] || (is_closed && !exterior[i]) {
                    occupancy[i] = Some(
                        occupancy[i].map_or(instance.owner, |owner| owner.min(instance.owner)),
                    );
                }
            }
        }
        let mut sections = [Vec::new(), Vec::new(), Vec::new()];
        for axis in 0..3 {
            sections[axis] = allocated(
                dimensions[axis],
                GeometrySection {
                    area: 0.0,
                    centroid: None,
                },
            )?;
        }
        let mut occupied_count = 0;
        for (i, owner) in occupancy.iter().enumerate() {
            if owner.is_none() {
                continue;
            }
            occupied_count += 1;
            let p = coordinates(i, dimensions);
            let center = origin + vector(p.map(|v| (v as f64 + 0.5) * h));
            for axis in 0..3 {
                let section = &mut sections[axis][p[axis]];
                // Running mean avoids coordinate sums overflowing at large offsets.
                let n = section.area;
                section.area += 1.0;
                section.centroid = Some(
                    section
                        .centroid
                        .map_or(center, |old| old + (center - old) / (n + 1.0)),
                );
            }
        }
        for axis in &mut sections {
            for section in axis {
                section.area *= h * h;
            }
        }
        Ok(Self {
            origin,
            cell_size: h,
            dimensions,
            occupancy,
            sections,
            volume: occupied_count as f64 * h * h * h,
        })
    }

    pub fn origin(&self) -> Vec3 {
        self.origin
    }
    pub fn cell_size(&self) -> f64 {
        self.cell_size
    }
    pub fn dimensions(&self) -> [usize; 3] {
        self.dimensions
    }
    pub fn occupancy(&self) -> &[Option<GeometryOwner>] {
        &self.occupancy
    }

    /// Transforms source triangles with a checked allocation and work ceiling.
    /// Each instance costs one unit and each triangle costs four units
    /// (three vertices and one normal/area).
    pub fn surface_triangles(
        instances: &[GeometryInstance<'_>],
        max_work: usize,
    ) -> Result<Vec<SurfaceTriangle>, crate::Error> {
        check_surface_work(Some(instances.len()), max_work)?;
        let total = surface_triangle_count(instances)?;
        check_surface_work(
            total
                .checked_mul(SURFACE_TRIANGLE_WORK)
                .and_then(|n| n.checked_add(instances.len())),
            max_work,
        )?;
        Self::collect_surface_triangles(instances, total)
    }

    fn collect_surface_triangles(
        instances: &[GeometryInstance<'_>],
        total: usize,
    ) -> Result<Vec<SurfaceTriangle>, crate::Error> {
        let mut triangles = Vec::new();
        triangles
            .try_reserve_exact(total)
            .map_err(|_| crate::Error::from(GeometryError::AllocationFailed))?;
        for instance in instances {
            for &[a, b, c] in instance.mesh.triangles() {
                let vertices = [a, b, c].map(|i| {
                    instance
                        .local_transform
                        .transform_point(instance.mesh.vertices()[i])
                });
                let cross = (vertices[1] - vertices[0]).cross(vertices[2] - vertices[0]);
                let length = cross
                    .checked_length()
                    .ok_or_else(|| crate::Error::from(GeometryError::ArithmeticOverflow))?;
                if length == 0.0 {
                    return Err(crate::Error::from(GeometryError::InvalidGeometry));
                }
                triangles.push(SurfaceTriangle {
                    vertices,
                    normal: cross / length,
                    area: length * 0.5,
                    owner: instance.owner,
                });
            }
        }
        Ok(triangles)
    }

    pub fn sections(&self) -> &[Vec<GeometrySection>; 3] {
        &self.sections
    }
    pub fn volume(&self) -> f64 {
        self.volume
    }
    /// Returns whether a body-local point lies in an occupied voxel.
    /// Grid occupancy is conservative and uses the configured open-mesh policy;
    /// this query is not an exact point-in-mesh test.
    pub fn occupied_at(&self, point: Vec3) -> Result<bool, crate::Error> {
        if !point.is_finite() {
            return Err(crate::Error::from(GeometryError::InvalidGeometry));
        }
        let upper = self.origin + vector(self.dimensions.map(|n| n as f64 * self.cell_size));
        if !upper.is_finite() {
            return Err(crate::Error::from(GeometryError::ArithmeticOverflow));
        }
        let point_components = components(point);
        let lower_components = components(self.origin);
        let upper_components = components(upper);
        if (0..3).any(|axis| {
            point_components[axis] < lower_components[axis]
                || point_components[axis] >= upper_components[axis]
        }) {
            return Ok(false);
        }
        let relative = components((point - self.origin) / self.cell_size);
        if relative.iter().any(|v| !v.is_finite() || *v < 0.0) {
            return Err(crate::Error::from(GeometryError::ArithmeticOverflow));
        }
        let cell = relative.map(|v| v.floor() as usize);
        if (0..3).any(|axis| cell[axis] >= self.dimensions[axis]) {
            return Err(crate::Error::from(GeometryError::PrecisionLoss));
        }
        self.occupancy
            .get(index(cell, self.dimensions))
            .map(Option::is_some)
            .ok_or_else(|| crate::Error::from(GeometryError::PrecisionLoss))
    }

    /// Partitions triangles along projected blocker edges and depth planes.
    /// Coplanar overlap belongs to the first canonical triangle (owner first).
    /// Source physical normals are preserved. Fully visible source triangles
    /// retain their centroid sampling; partial polygons are triangulated.
    /// Affine clipping uses adaptive orientation predicates and rejects unsupported
    /// predicate ranges or unstable area partitions with `PrecisionLoss`.
    /// Work exhaustion rejects the entire result; allocation is fallible.
    /// `epsilon` is validated for compatibility and never offsets shadow rays.
    pub fn exposed_surface_patches(
        &self,
        instances: &[GeometryInstance<'_>],
        direction: Vec3,
        epsilon: f64,
        max_work: usize,
    ) -> Result<PreparedSurfacePatches, crate::Error> {
        if !direction.is_finite() || !epsilon.is_finite() || epsilon <= 0.0 {
            return Err(crate::Error::from(GeometryError::InvalidGeometry));
        }
        let upstream = direction
            .try_normalize()
            .ok_or_else(|| crate::Error::from(GeometryError::InvalidGeometry))?;
        self.surface_patches_with_flow(instances, max_work, |_| Ok(upstream))
    }

    /// Centroid flow quadrature: each source triangle has its own upstream ray.
    /// The callback must be affine in position (rigid motion, uniform wind).
    /// Windward sign changes are clipped before choosing a representative ray.
    /// Mesh refinement is required when rotational flow varies within a triangle.
    pub(crate) fn surface_patches_with_flow(
        &self,
        instances: &[GeometryInstance<'_>],
        max_work: usize,
        flow: impl Fn(Vec3) -> Result<Vec3, crate::Error>,
    ) -> Result<PreparedSurfacePatches, crate::Error> {
        // Counting, selecting a reference, constructing relative instances
        // and collecting triangles each visit every instance.
        check_surface_work(instances.len().checked_mul(4), max_work)?;
        let total = surface_triangle_count(instances)?;
        let preprocessing = sort_work_estimate(total)
            .and_then(|n| n.checked_mul(TRIANGLE_PREPARATION_WORK))
            .and_then(|n| {
                instances
                    .len()
                    .checked_mul(4)
                    .and_then(|i| n.checked_add(i))
            });
        check_surface_work(preprocessing, max_work)?;
        let work =
            preprocessing.ok_or_else(|| crate::Error::from(GeometryError::WorkBudgetExceeded))?;
        let Some(reference) = instances
            .iter()
            .min_by(|a, b| {
                compare_vertex(
                    &a.local_transform.translation(),
                    &b.local_transform.translation(),
                )
                .then_with(|| {
                    let (a, b) = (a.local_transform.rotation(), b.local_transform.rotation());
                    compare_vertex(&Vec3::new(a.x, a.y, a.z), &Vec3::new(b.x, b.y, b.z))
                        .then_with(|| a.w.total_cmp(&b.w))
                })
            })
            .map(|instance| instance.local_transform)
        else {
            return Ok(PreparedSurfacePatches {
                patches: Vec::new(),
            });
        };
        let inverse_rotation = reference.rotation().conjugate();
        let mut relative = Vec::new();
        relative
            .try_reserve_exact(instances.len())
            .map_err(|_| crate::Error::from(GeometryError::AllocationFailed))?;
        for instance in instances {
            let local_transform = if instance.local_transform == reference {
                Transform::identity()
            } else {
                let delta = instance
                    .local_transform
                    .translation()
                    .checked_sub(reference.translation())
                    .ok_or_else(|| crate::Error::from(GeometryError::ArithmeticOverflow))?;
                let translation = inverse_rotation
                    .rotate_vector(delta)
                    .ok_or_else(|| crate::Error::from(GeometryError::ArithmeticOverflow))?;
                let rotation = if instance.local_transform.rotation() == reference.rotation() {
                    kapitsa_math::Quat::IDENTITY
                } else {
                    (inverse_rotation * instance.local_transform.rotation())
                        .try_normalize()
                        .ok_or_else(|| crate::Error::from(GeometryError::ArithmeticOverflow))?
                };
                Transform::new(translation, rotation)
                    .map_err(|_| crate::Error::from(GeometryError::ArithmeticOverflow))?
            };
            relative.push(GeometryInstance {
                local_transform,
                ..*instance
            });
        }
        let mut patches =
            self.surface_patches_in_frame(&relative, total, max_work, work, |point| {
                let point = reference.transform_point(point);
                if !point.is_finite() {
                    return Err(crate::Error::from(GeometryError::ArithmeticOverflow));
                }
                inverse_rotation
                    .rotate_vector(flow(point)?)
                    .ok_or_else(|| crate::Error::from(GeometryError::ArithmeticOverflow))
            })?;
        for patch in &mut patches.patches {
            patch.vertices = patch.vertices.map(|point| reference.transform_point(point));
            patch.center = reference.transform_point(patch.center);
            patch.normal = reference.transform_vector(patch.normal);
            if !patch.vertices.iter().all(|p| p.is_finite())
                || !patch.center.is_finite()
                || !patch.normal.is_finite()
            {
                return Err(crate::Error::from(GeometryError::ArithmeticOverflow));
            }
        }
        Ok(patches)
    }

    fn surface_patches_in_frame(
        &self,
        instances: &[GeometryInstance<'_>],
        total: usize,
        max_work: usize,
        mut work: usize,
        flow: impl Fn(Vec3) -> Result<Vec3, crate::Error>,
    ) -> Result<PreparedSurfacePatches, crate::Error> {
        // Work units are conservative accounting weights, not instruction counts.
        const SOURCE_WORK: usize = 48;
        const SHADOW_WORK: usize = 64;
        const CLIP_VERTEX_WORK: usize = 16;
        const POLYGON_VISIT_WORK: usize = 1;
        const PARTITION_TRIANGLE_WORK: usize = 16;
        const REPLACEMENT_PATCH_WORK: usize = 1;
        // Reserve the outer frame's vertex, center and normal transformations.
        const PATCH_FRAME_WORK: usize = 1;

        // Grid operations must receive the remainder of the same hard budget
        // that is used to charge their work after they return.
        let max_work = max_work.min(MAX_SURFACE_WORK);
        let mut triangles = Self::collect_surface_triangles(instances, total)?;
        // Exact canonical vertex triples avoid centroid/area aliasing and
        // many-to-one epsilon matching. Winding remains in the source normal.
        for triangle in &mut triangles {
            triangle.vertices.sort_unstable_by(compare_vertex);
        }
        // Equal keys describe identical prepared triangles. Sorting is in-place
        // and comparison work is charged before each comparator invocation.
        budgeted_sort(&mut triangles, &mut work, max_work, |a, b| {
            a.owner
                .cmp(&b.owner)
                .then_with(|| {
                    a.vertices
                        .iter()
                        .zip(&b.vertices)
                        .map(|(a, b)| compare_vertex(a, b))
                        .find(|order| !order.is_eq())
                        .unwrap_or(std::cmp::Ordering::Equal)
                })
                .then_with(|| compare_vertex(&a.normal, &b.normal))
                .then_with(|| a.area.total_cmp(&b.area))
        })?;
        let grid = TriangleGrid::build(&triangles, max_work.saturating_sub(work))?;
        work = work
            .checked_add(grid.build_work)
            .ok_or_else(|| crate::Error::from(GeometryError::WorkBudgetExceeded))?;
        check_surface_work(Some(work), max_work)?;
        let mut marks = allocate_candidate_marks(triangles.len(), &mut work, max_work)?;
        let mut stamp = 0u32;
        let mut patches: Vec<PreparedSurfacePatch> = Vec::new();
        for (i, triangle) in triangles.iter().enumerate() {
            let [a, b, c] = triangle.vertices;
            work = work
                .checked_add(SOURCE_WORK)
                .ok_or_else(|| crate::Error::from(GeometryError::WorkBudgetExceeded))?;
            check_surface_work(Some(work), max_work)?;
            let velocities = [flow(a)?, flow(b)?, flow(c)?];
            let Some(initial) = visibility::windward_triangle(triangle.normal, velocities)? else {
                continue;
            };
            let initial_fraction = visibility::polygon_fraction(&initial, |amount| {
                charge_surface_work(&mut work, amount, max_work)
            })?;
            if initial_fraction == 0.0 {
                return Err(crate::Error::from(GeometryError::PrecisionLoss));
            }
            let centroid = initial
                .iter()
                .copied()
                .fold(Vec3::ZERO, |sum, point| sum + point)
                / (initial.len() as f64);
            let weights = [1.0 - centroid.x - centroid.y, centroid.x, centroid.y];
            if weights.iter().any(|weight| *weight < 0.0) {
                return Err(crate::Error::from(GeometryError::PrecisionLoss));
            }
            let mut velocity = Vec3::ZERO;
            for (value, weight) in velocities.into_iter().zip(weights) {
                velocity = value
                    .checked_mul_scalar(weight)
                    .and_then(|v| velocity.checked_add(v))
                    .ok_or_else(|| crate::Error::from(GeometryError::ArithmeticOverflow))?;
            }
            let upstream = velocity
                .try_normalize()
                .ok_or_else(|| crate::Error::from(GeometryError::ArithmeticOverflow))?;
            if triangle
                .normal
                .checked_dot(upstream)
                .ok_or_else(|| crate::Error::from(GeometryError::ArithmeticOverflow))?
                <= 0.0
            {
                return Err(crate::Error::from(GeometryError::PrecisionLoss));
            }
            let first_patch = patches.len();
            let mut any_blocked = false;
            let mut pieces = Vec::new();
            checked_push(&mut pieces, initial)?;
            let mut shadows = Vec::new();
            stamp = stamp
                .checked_add(1)
                .ok_or_else(|| crate::Error::from(GeometryError::WorkBudgetExceeded))?;
            let (candidates, query_work) = grid.candidates(
                triangle,
                upstream,
                &mut marks,
                stamp,
                max_work.saturating_sub(work),
            )?;
            work = work
                .checked_add(query_work)
                .ok_or_else(|| crate::Error::from(GeometryError::WorkBudgetExceeded))?;
            check_surface_work(Some(work), max_work)?;
            let partition_start_work = work;
            let mut exact_work = 0;
            let mut next = Vec::new();
            let mut distances = Vec::new();
            for j in candidates {
                if i == j {
                    continue;
                }
                // Shadow construction evaluates multiple adaptive orientation
                // predicates even when the candidate produces no shadow.
                work = work
                    .checked_add(SHADOW_WORK)
                    .ok_or_else(|| crate::Error::from(GeometryError::WorkBudgetExceeded))?;
                check_surface_work(Some(work), max_work)?;
                let blocker = &triangles[j];
                let Some(shadow) =
                    visibility::Shadow::new(triangle, blocker, i, j, upstream, |amount| {
                        charge_surface_work(&mut work, amount, max_work)?;
                        exact_work += amount;
                        Ok(())
                    })?
                else {
                    continue;
                };
                for coefficients in shadow.planes {
                    if !coefficients.iter().any(|value| *value > 0.0)
                        || !coefficients.iter().any(|value| *value < 0.0)
                    {
                        continue;
                    }
                    for polygon in pieces.drain(..) {
                        let polygon_work = polygon
                            .len()
                            .checked_mul(CLIP_VERTEX_WORK)
                            .and_then(|n| n.checked_add(POLYGON_VISIT_WORK))
                            .ok_or_else(|| crate::Error::from(GeometryError::WorkBudgetExceeded))?;
                        charge_surface_work(&mut work, polygon_work, max_work)?;
                        distances.clear();
                        distances
                            .try_reserve_exact(polygon.len())
                            .map_err(|_| crate::Error::from(GeometryError::AllocationFailed))?;
                        let crosses = |a: f64, b: f64| (a > 0.0 && b < 0.0) || (a < 0.0 && b > 0.0);
                        let mut positive_count = 0;
                        let mut negative_count = 0;
                        let mut crossings = 0;
                        let mut previous = 0.0;
                        for point in &polygon {
                            let distance = visibility::distance(coefficients, *point, |amount| {
                                charge_surface_work(&mut work, amount, max_work)?;
                                exact_work += amount;
                                Ok(())
                            })?;
                            positive_count += usize::from(distance > 0.0);
                            negative_count += usize::from(distance < 0.0);
                            crossings += usize::from(crosses(previous, distance));
                            previous = distance;
                            distances.push(distance);
                        }
                        if positive_count == 0 || negative_count == 0 {
                            checked_push(&mut next, polygon)?;
                            continue;
                        }
                        crossings += usize::from(crosses(previous, distances[0]));
                        // The precharged vertex count bounds both reservations:
                        // each side keeps at most n vertices and adds at most n crossings.
                        let mut positive = Vec::new();
                        let mut negative = Vec::new();
                        positive
                            .try_reserve_exact(polygon.len() - negative_count + crossings)
                            .map_err(|_| crate::Error::from(GeometryError::AllocationFailed))?;
                        negative
                            .try_reserve_exact(polygon.len() - positive_count + crossings)
                            .map_err(|_| crate::Error::from(GeometryError::AllocationFailed))?;
                        for k in 0..polygon.len() {
                            let n = (k + 1) % polygon.len();
                            let (d, e) = (distances[k], distances[n]);
                            if d >= 0.0 {
                                positive.push(polygon[k]);
                            }
                            if d <= 0.0 {
                                negative.push(polygon[k]);
                            }
                            if crosses(d, e) {
                                let point =
                                    visibility::crossing_point(polygon[k], polygon[n], d, e)?;
                                positive.push(point);
                                negative.push(point);
                            }
                        }
                        checked_push(&mut next, positive)?;
                        checked_push(&mut next, negative)?;
                    }
                    std::mem::swap(&mut pieces, &mut next);
                }
                checked_push(&mut shadows, shadow)?;
            }
            let mut partition_fraction = visibility::AreaSum::default();
            for polygon in pieces {
                charge_surface_work(&mut work, POLYGON_VISIT_WORK, max_work)?;
                for k in 1..polygon.len().saturating_sub(1) {
                    // Partition reconstruction and output remain work even
                    // when there are no shadows to test against the centroid.
                    work = work
                        .checked_add(PARTITION_TRIANGLE_WORK)
                        .ok_or_else(|| crate::Error::from(GeometryError::WorkBudgetExceeded))?;
                    check_surface_work(Some(work), max_work)?;
                    let coordinates = [polygon[0], polygon[k], polygon[k + 1]];
                    let fraction = visibility::triangle_fraction(coordinates, |amount| {
                        charge_surface_work(&mut work, amount, max_work)?;
                        exact_work += amount;
                        Ok(())
                    })?;
                    partition_fraction.add(fraction);
                    let area = triangle.area * fraction;
                    let [a, b, c] = triangle.vertices;
                    let reconstruct = |point: Vec3| a + (b - a) * point.x + (c - a) * point.y;
                    let vertices = coordinates.map(reconstruct);
                    if !area.is_finite() {
                        return Err(crate::Error::from(GeometryError::ArithmeticOverflow));
                    }
                    if area == 0.0 {
                        if fraction != 0.0 {
                            return Err(crate::Error::from(GeometryError::PrecisionLoss));
                        }
                        continue;
                    }
                    let centroid = coordinates[0]
                        + (coordinates[1] - coordinates[0]) / 3.0
                        + (coordinates[2] - coordinates[0]) / 3.0;
                    let center = reconstruct(centroid);
                    if !center.is_finite() {
                        return Err(crate::Error::from(GeometryError::ArithmeticOverflow));
                    }
                    let mut blocked = false;
                    for shadow in &shadows {
                        work = work
                            .checked_add(1)
                            .ok_or_else(|| crate::Error::from(GeometryError::WorkBudgetExceeded))?;
                        check_surface_work(Some(work), max_work)?;
                        if shadow.covers(centroid, |amount| {
                            charge_surface_work(&mut work, amount, max_work)?;
                            exact_work += amount;
                            Ok(())
                        })? {
                            blocked = true;
                            break;
                        }
                    }
                    any_blocked |= blocked;
                    if !blocked {
                        checked_push(
                            &mut patches,
                            PreparedSurfacePatch {
                                vertices,
                                center,
                                normal: triangle.normal,
                                area,
                                owner: triangle.owner,
                            },
                        )?;
                    }
                }
            }
            // Splitting must conserve the source's unit barycentric area.
            // Heuristic consistency tolerance based on this source partition.
            // Exact evaluation does not accumulate floating-point roundoff.
            // The initial triangle contributes one unit even before clipping.
            let partition_work = work - partition_start_work - exact_work;
            let roundoff = (partition_work as f64 + 1.0) * f64::EPSILON;
            let allowance = roundoff / (1.0 - roundoff);
            let partition_fraction = partition_fraction.value();
            if !partition_fraction.is_finite()
                || (partition_fraction - initial_fraction).abs() > allowance
            {
                return Err(crate::Error::from(GeometryError::PrecisionLoss));
            }
            if !any_blocked && initial_fraction == 1.0 {
                // Replacing the partition constructs an additional output patch.
                charge_surface_work(&mut work, REPLACEMENT_PATCH_WORK, max_work)?;
                patches.truncate(first_patch);
                let [a, b, c] = triangle.vertices;
                checked_push(
                    &mut patches,
                    PreparedSurfacePatch {
                        vertices: triangle.vertices,
                        center: a + (b - a) / 3.0 + (c - a) / 3.0,
                        normal: triangle.normal,
                        area: triangle.area,
                        owner: triangle.owner,
                    },
                )?;
            }
        }
        let frame_work = patches
            .len()
            .checked_mul(PATCH_FRAME_WORK)
            .ok_or_else(|| crate::Error::from(GeometryError::WorkBudgetExceeded))?;
        charge_surface_work(&mut work, frame_work, max_work)?;
        Ok(PreparedSurfacePatches { patches })
    }

    /// Streams union boundary faces without allocating a worst-case face buffer.
    pub fn exposed_faces(&self) -> impl Iterator<Item = GeometryFace> + '_ {
        self.occupancy
            .iter()
            .enumerate()
            .filter_map(|(i, owner)| owner.map(|o| (i, o)))
            .flat_map(move |(i, owner)| {
                let p = coordinates(i, self.dimensions);
                (0..3).flat_map(move |axis| {
                    [false, true].into_iter().filter_map(move |positive| {
                        if neighbor(p, axis, positive, self.dimensions)
                            .is_some_and(|n| self.occupancy[n].is_some())
                        {
                            return None;
                        }
                        let mut offset = [0.0; 3];
                        offset[axis] = if positive { 0.5 } else { -0.5 };
                        let normal = vector(offset) * 2.0;
                        Some(GeometryFace {
                            center: self.origin
                                + vector(p.map(|v| (v as f64 + 0.5) * self.cell_size))
                                + vector(offset) * self.cell_size,
                            normal,
                            area: self.cell_size * self.cell_size,
                            owner,
                        })
                    })
                })
            })
    }
}

#[cfg(test)]
#[path = "geometry_tests.rs"]
mod tests;
