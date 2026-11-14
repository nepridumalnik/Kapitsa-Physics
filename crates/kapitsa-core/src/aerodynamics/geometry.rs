//! Scalar conservative voxel geometry, independent of forces and center of mass.
//!
//! Cells intersecting a triangle are occupied in full. Thin surfaces survive, but
//! volume and surface area are resolution dependent. Voxel normals are axis-aligned:
//! they are not mesh normals and must not be used as a convergent pressure model.
//! Closed instances use solid flood-fill semantics (sealed cavities are filled).
//! Indexed seams must be welded; self-intersections are not diagnosed.

use std::fmt;

use kapitsa_math::Vec3;

use crate::{shape::TriangleMesh, transform::Transform, world::PartId};

/// Attribution within one body's local frame. Lowest owner wins overlap ties.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum GeometryOwner {
    Body,
    Part(PartId),
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

fn allocated<T: Clone>(len: usize, value: T) -> Result<Vec<T>, GeometryError> {
    let mut values = Vec::new();
    values
        .try_reserve_exact(len)
        .map_err(|_| GeometryError::AllocationFailed)?;
    values.resize(len, value);
    Ok(values)
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

fn closed(mesh: &TriangleMesh, weld_tolerance: Option<f64>) -> Result<bool, GeometryError> {
    let welded = if let Some(tolerance) = weld_tolerance {
        if !tolerance.is_finite() || tolerance <= 0.0 {
            return Err(GeometryError::InvalidConfig);
        }
        let mut representatives: Vec<usize> = Vec::new();
        representatives
            .try_reserve_exact(mesh.vertices().len())
            .map_err(|_| GeometryError::AllocationFailed)?;
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
                .ok_or(GeometryError::WorkBudgetExceeded)?,
        )
        .map_err(|_| GeometryError::AllocationFailed)?;
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
        return Err(GeometryError::InvalidGeometry);
    }
    let mut volume = 0.0;
    for &[a, b, c] in mesh.triangles() {
        let [a, b, c] = [a, b, c].map(|i| (mesh.vertices()[i] - reference) / scale);
        volume += a.dot(b.cross(c));
    }
    Ok(volume.is_finite() && volume != 0.0)
}

/// Separating-axis triangle/box test: box axes, triangle normal, nine edge axes.
fn intersects(triangle: [Vec3; 3], center: Vec3, half: f64) -> Result<bool, GeometryError> {
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
            return Err(GeometryError::ArithmeticOverflow);
        }
        if scale == 0.0 {
            continue;
        }
        let axis = axis / scale;
        let projected = v.map(|p| p.dot(axis));
        let radius = half * (axis.x.abs() + axis.y.abs() + axis.z.abs());
        if !radius.is_finite() || !projected.iter().all(|v| v.is_finite()) {
            return Err(GeometryError::ArithmeticOverflow);
        }
        // Include boundary contact so grid-aligned zero-thickness surfaces survive.
        if projected.iter().all(|&v| v > radius) || projected.iter().all(|&v| v < -radius) {
            return Ok(false);
        }
    }
    Ok(true)
}

impl PreparedGeometry {
    pub fn prepare(
        instances: &[GeometryInstance<'_>],
        config: GeometryConfig,
    ) -> Result<Self, GeometryError> {
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
            return Err(GeometryError::InvalidConfig);
        }
        if instances.is_empty() {
            return Err(GeometryError::EmptyGeometry);
        }
        let triangle_count = instances
            .iter()
            .try_fold(0usize, |n, i| n.checked_add(i.mesh.triangles().len()))
            .ok_or(GeometryError::WorkBudgetExceeded)?;
        let vertex_count = instances
            .iter()
            .try_fold(0usize, |n, i| n.checked_add(i.mesh.vertices().len()))
            .ok_or(GeometryError::WorkBudgetExceeded)?;
        if triangle_count
            .checked_add(vertex_count)
            .is_none_or(|n| n > config.max_work)
        {
            return Err(GeometryError::WorkBudgetExceeded);
        }
        let mut min = Vec3::splat(f64::INFINITY);
        let mut max = Vec3::splat(f64::NEG_INFINITY);
        for instance in instances {
            for triangle in instance.mesh.triangles() {
                for &index in triangle {
                    let vertex = instance.mesh.vertices()[index];
                    let v = instance.local_transform.transform_point(vertex);
                    if !v.is_finite() {
                        return Err(GeometryError::ArithmeticOverflow);
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
            return Err(GeometryError::CellBudgetExceeded);
        }
        if (origin + Vec3::splat(h * 0.5)).x == origin.x
            || (origin + Vec3::splat(h * 0.5)).y == origin.y
            || (origin + Vec3::splat(h * 0.5)).z == origin.z
        {
            return Err(GeometryError::InvalidGeometry);
        }
        let dimensions = extents.map(|x| (x.ceil() as usize).max(1) + 2);
        let count = dimensions
            .into_iter()
            .try_fold(1usize, |n, d| n.checked_mul(d))
            .ok_or(GeometryError::CellBudgetExceeded)?;
        if count > config.max_cells {
            return Err(GeometryError::CellBudgetExceeded);
        }
        let work = count
            .checked_mul(
                triangle_count
                    .checked_add(instances.len().saturating_mul(8))
                    .ok_or(GeometryError::WorkBudgetExceeded)?,
            )
            .ok_or(GeometryError::WorkBudgetExceeded)?;
        if work
            .checked_add(vertex_count)
            .is_none_or(|n| n > config.max_work)
        {
            return Err(GeometryError::WorkBudgetExceeded);
        }
        let upper = origin + vector(dimensions.map(|d| d as f64 * h));
        if !upper.is_finite() {
            return Err(GeometryError::ArithmeticOverflow);
        }
        if !(h * h * count as f64 * 6.0).is_finite() || !(h * h * h * count as f64).is_finite() {
            return Err(GeometryError::ArithmeticOverflow);
        }
        let mut occupancy: Vec<Option<GeometryOwner>> = allocated(count, None)?;
        let mut shell = allocated(count, false)?;
        let mut exterior = allocated(count, false)?;
        let mut queue = Vec::new();
        queue
            .try_reserve_exact(count)
            .map_err(|_| GeometryError::AllocationFailed)?;
        for instance in instances {
            let is_closed = closed(instance.mesh, config.weld_tolerance)?;
            if !is_closed && config.open_mesh_policy == OpenMeshPolicy::Reject {
                return Err(GeometryError::OpenMesh);
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
                let triangle_min = triangle
                    .iter()
                    .fold(Vec3::splat(f64::INFINITY), |min, &v| min.min(v));
                let triangle_max = triangle
                    .iter()
                    .fold(Vec3::splat(f64::NEG_INFINITY), |max, &v| max.max(v));
                let lower = components((triangle_min - origin) / h)
                    .map(|value| value.floor().max(0.0) as usize);
                let upper = components((triangle_max - origin) / h)
                    .map(|value| value.ceil().max(0.0) as usize);
                let lower = [
                    lower[0].min(dimensions[0].saturating_sub(1)),
                    lower[1].min(dimensions[1].saturating_sub(1)),
                    lower[2].min(dimensions[2].saturating_sub(1)),
                ];
                let upper = [
                    upper[0].min(dimensions[0].saturating_sub(1)),
                    upper[1].min(dimensions[1].saturating_sub(1)),
                    upper[2].min(dimensions[2].saturating_sub(1)),
                ];
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
                let mut head = 0;
                while head < queue.len() {
                    let p = coordinates(queue[head], dimensions);
                    head += 1;
                    for axis in 0..3 {
                        for positive in [false, true] {
                            if let Some(i) = neighbor(p, axis, positive, dimensions)
                                && !shell[i]
                                && !exterior[i]
                            {
                                exterior[i] = true;
                                queue.push(i);
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

    pub fn surface_triangles(
        instances: &[GeometryInstance<'_>],
    ) -> Result<Vec<SurfaceTriangle>, GeometryError> {
        let mut triangles = Vec::new();
        let total = instances
            .iter()
            .try_fold(0usize, |n, instance| {
                n.checked_add(instance.mesh.triangles().len())
            })
            .ok_or(GeometryError::WorkBudgetExceeded)?;
        triangles
            .try_reserve_exact(total)
            .map_err(|_| GeometryError::AllocationFailed)?;
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
                    .ok_or(GeometryError::ArithmeticOverflow)?;
                if length == 0.0 {
                    return Err(GeometryError::InvalidGeometry);
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

    pub fn exposed_surface_patches(
        &self,
        instances: &[GeometryInstance<'_>],
        direction: Vec3,
        epsilon: f64,
        max_work: usize,
    ) -> Result<PreparedSurfacePatches, GeometryError> {
        if !direction.is_finite() || !epsilon.is_finite() || epsilon <= 0.0 {
            return Err(GeometryError::InvalidGeometry);
        }
        // Direction points from a patch toward its upstream/oncoming source.
        // The fluid velocity itself is the opposite direction.
        let upstream = direction
            .try_normalize()
            .ok_or(GeometryError::InvalidGeometry)?;
        let triangles = Self::surface_triangles(instances)?;
        let projected_work = triangles
            .len()
            .checked_mul(self.dimensions.iter().sum::<usize>())
            .ok_or(GeometryError::WorkBudgetExceeded)?;
        if max_work == 0 || projected_work > max_work {
            return Err(GeometryError::WorkBudgetExceeded);
        }
        let mut patches = Vec::new();
        patches
            .try_reserve_exact(triangles.len())
            .map_err(|_| GeometryError::AllocationFailed)?;
        let mut work = 0usize;
        for triangle in triangles {
            let center = (triangle.vertices[0] + triangle.vertices[1] + triangle.vertices[2]) / 3.0;
            if triangle.normal.dot(upstream) <= 0.0 {
                continue;
            }
            work = work
                .checked_add(self.dimensions.iter().sum::<usize>())
                .ok_or(GeometryError::WorkBudgetExceeded)?;
            if work > max_work {
                return Err(GeometryError::WorkBudgetExceeded);
            }
            if self.ray_hits_union(center, -upstream, epsilon) {
                continue;
            }
            let duplicate = patches.iter().position(|patch: &PreparedSurfacePatch| {
                patch.vertices.iter().all(|vertex| {
                    triangle.vertices.iter().any(|candidate| {
                        (*vertex - *candidate).length_squared() <= epsilon * epsilon
                    })
                })
            });
            if let Some(index) = duplicate {
                let owner = patches[index].owner.min(triangle.owner);
                patches[index].owner = owner;
            } else {
                patches.push(PreparedSurfacePatch {
                    vertices: triangle.vertices,
                    center,
                    normal: triangle.normal,
                    area: triangle.area,
                    owner: triangle.owner,
                });
            }
        }
        Ok(PreparedSurfacePatches { patches })
    }

    fn ray_hits_union(&self, point: Vec3, direction: Vec3, epsilon: f64) -> bool {
        let start = point + direction * (epsilon + self.cell_size);
        let max_steps = self.dimensions[0]
            .saturating_add(self.dimensions[1])
            .saturating_add(self.dimensions[2])
            .saturating_add(4);
        for step in 0..max_steps {
            let sample = start + direction * (step as f64 * self.cell_size);
            let relative = (sample - self.origin) / self.cell_size;
            let coordinates =
                [relative.x, relative.y, relative.z].map(|value| value.floor() as isize);
            if coordinates.iter().any(|&value| value < 0) {
                return false;
            }
            let coordinates = coordinates.map(|value| value as usize);
            if coordinates
                .iter()
                .enumerate()
                .any(|(axis, &value)| value >= self.dimensions[axis])
            {
                return false;
            }
            if self.occupancy[index(coordinates, self.dimensions)].is_some() {
                return true;
            }
        }
        false
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
