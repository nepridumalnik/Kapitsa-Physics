use crate::{shape::Shape, transform::Transform};
use kapitsa_math::Vec3;

use super::{CollisionError, gjk::Simplex, support::SupportPoint, support::support_pair};

const MAX_EPA_ITERATIONS: usize = 64;

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct EpaResult {
    pub(crate) normal: Vec3,
    pub(crate) penetration: f64,
    pub(crate) point_a: Vec3,
    pub(crate) point_b: Vec3,
    pub(crate) feature_a: u64,
    pub(crate) feature_b: u64,
}

#[derive(Clone, Copy)]
struct Face {
    vertices: [usize; 3],
    normal: Vec3,
    distance: f64,
}

/// Reusable scratch for bounded EPA expansion.
pub(crate) struct EpaWorkspace {
    vertices: Vec<SupportPoint>,
    faces: Vec<Face>,
    visible: Vec<usize>,
    horizon: Vec<(usize, usize)>,
}

impl EpaWorkspace {
    pub(crate) const fn new() -> Self {
        Self {
            vertices: Vec::new(),
            faces: Vec::new(),
            visible: Vec::new(),
            horizon: Vec::new(),
        }
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn retained_capacity_bytes(&self) -> usize {
        use std::mem::size_of;

        self.vertices.capacity() * size_of::<SupportPoint>()
            + self.faces.capacity() * size_of::<Face>()
            + self.visible.capacity() * size_of::<usize>()
            + self.horizon.capacity() * size_of::<(usize, usize)>()
    }

    pub(crate) fn solve(
        &mut self,
        simplex: &Simplex,
        shape_a: &Shape,
        transform_a: &Transform,
        shape_b: &Shape,
        transform_b: &Transform,
    ) -> Result<EpaResult, CollisionError> {
        if simplex.count != 4 {
            return Err(CollisionError::NonConvergent);
        }
        self.vertices.clear();
        self.faces.clear();
        self.visible.clear();
        self.horizon.clear();
        self.vertices
            .try_reserve(4)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        self.faces
            .try_reserve(8)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        self.vertices.extend_from_slice(&simplex.points);
        for vertices in [[0, 1, 2], [0, 3, 1], [0, 2, 3], [1, 3, 2]] {
            let face = self.make_face(vertices)?;
            self.faces.push(face);
        }

        for _ in 0..MAX_EPA_ITERATIONS {
            let (_, face) = self.closest_face()?;
            let support = support_pair(shape_a, transform_a, shape_b, transform_b, face.normal)?;
            let projection = support
                .difference
                .checked_dot(face.normal)
                .ok_or(CollisionError::ArithmeticOverflow)?;
            let tolerance = scale_tolerance(support.difference);
            if projection - face.distance <= tolerance {
                return self.contact(face);
            }

            let new_index = self.vertices.len();
            self.vertices
                .try_reserve(1)
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            self.visible.clear();
            self.horizon.clear();
            for (index, face) in self.faces.iter().copied().enumerate() {
                let origin = self.vertex(face.vertices[0])?.difference;
                let offset = support
                    .difference
                    .checked_sub(origin)
                    .ok_or(CollisionError::ArithmeticOverflow)?;
                if face.normal.dot(offset) > tolerance {
                    self.visible
                        .try_reserve(1)
                        .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
                    self.visible.push(index);
                }
            }
            if self.visible.is_empty() {
                return Err(CollisionError::NonConvergent);
            }
            for position in 0..self.visible.len() {
                let index = self
                    .visible
                    .get(position)
                    .copied()
                    .ok_or(CollisionError::InvalidContact)?;
                let face = self
                    .faces
                    .get(index)
                    .copied()
                    .ok_or(CollisionError::InvalidContact)?;
                self.add_horizon_edge(face.vertices[0], face.vertices[1])?;
                self.add_horizon_edge(face.vertices[1], face.vertices[2])?;
                self.add_horizon_edge(face.vertices[2], face.vertices[0])?;
            }
            if self.horizon.is_empty() {
                return Err(CollisionError::NonConvergent);
            }
            self.faces
                .try_reserve(self.horizon.len())
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            self.vertices.push(support);
            for index in self.visible.iter().rev().copied() {
                if index >= self.faces.len() {
                    return Err(CollisionError::InvalidContact);
                }
                self.faces.swap_remove(index);
            }
            for (from, to) in self.horizon.iter().copied() {
                self.faces.push(self.make_face([from, to, new_index])?);
            }
        }
        Err(CollisionError::NonConvergent)
    }

    fn vertex(&self, index: usize) -> Result<SupportPoint, CollisionError> {
        self.vertices
            .get(index)
            .copied()
            .ok_or(CollisionError::InvalidContact)
    }

    fn make_face(&self, mut vertices: [usize; 3]) -> Result<Face, CollisionError> {
        let a = self.vertex(vertices[0])?.difference;
        let b = self.vertex(vertices[1])?.difference;
        let c = self.vertex(vertices[2])?.difference;
        let normal = (b - a)
            .checked_cross(c - a)
            .and_then(Vec3::try_normalize)
            .ok_or(CollisionError::NonConvergent)?;
        let mut distance = normal
            .checked_dot(a)
            .ok_or(CollisionError::ArithmeticOverflow)?;
        let normal = if distance < 0.0 {
            vertices.swap(1, 2);
            distance = -distance;
            -normal
        } else {
            normal
        };
        Ok(Face {
            vertices,
            normal,
            distance,
        })
    }

    fn closest_face(&self) -> Result<(usize, Face), CollisionError> {
        let mut faces = self.faces.iter().copied().enumerate();
        let first = faces.next().ok_or(CollisionError::NonConvergent)?;
        let mut closest = first;
        for candidate in faces {
            if candidate.1.distance < closest.1.distance {
                closest = candidate;
            }
        }
        Ok(closest)
    }

    fn add_horizon_edge(&mut self, from: usize, to: usize) -> Result<(), CollisionError> {
        if let Some(position) = self
            .horizon
            .iter()
            .position(|&(edge_from, edge_to)| edge_from == to && edge_to == from)
        {
            self.horizon.swap_remove(position);
        } else {
            self.horizon
                .try_reserve(1)
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            self.horizon.push((from, to));
        }
        Ok(())
    }

    fn contact(&self, face: Face) -> Result<EpaResult, CollisionError> {
        let a = self.vertex(face.vertices[0])?;
        let b = self.vertex(face.vertices[1])?;
        let c = self.vertex(face.vertices[2])?;
        let target = face
            .normal
            .checked_mul_scalar(face.distance)
            .ok_or(CollisionError::ArithmeticOverflow)?;
        let weights = barycentric(target, a.difference, b.difference, c.difference)?;
        let point_a = weighted_point(weights, a.point_a, b.point_a, c.point_a)?;
        let point_b = weighted_point(weights, a.point_b, b.point_b, c.point_b)?;
        let (feature_a, feature_b) = if weights[0] >= weights[1] && weights[0] >= weights[2] {
            (a.feature_a, a.feature_b)
        } else if weights[1] >= weights[2] {
            (b.feature_a, b.feature_b)
        } else {
            (c.feature_a, c.feature_b)
        };
        Ok(EpaResult {
            normal: face.normal,
            penetration: face.distance,
            point_a,
            point_b,
            feature_a,
            feature_b,
        })
    }
}

fn barycentric(point: Vec3, a: Vec3, b: Vec3, c: Vec3) -> Result<[f64; 3], CollisionError> {
    let v0 = b.checked_sub(a).ok_or(CollisionError::ArithmeticOverflow)?;
    let v1 = c.checked_sub(a).ok_or(CollisionError::ArithmeticOverflow)?;
    let v2 = point
        .checked_sub(a)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let d00 = v0
        .checked_dot(v0)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let d01 = v0
        .checked_dot(v1)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let d11 = v1
        .checked_dot(v1)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let d20 = v2
        .checked_dot(v0)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let d21 = v2
        .checked_dot(v1)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let denominator = d00 * d11 - d01 * d01;
    if !denominator.is_finite() || denominator <= f64::MIN_POSITIVE {
        return Err(CollisionError::NonConvergent);
    }
    let mut v = (d11 * d20 - d01 * d21) / denominator;
    let mut w = (d00 * d21 - d01 * d20) / denominator;
    let mut u = 1.0 - v - w;
    if !u.is_finite() || !v.is_finite() || !w.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }
    u = u.clamp(0.0, 1.0);
    v = v.clamp(0.0, 1.0);
    w = w.clamp(0.0, 1.0);
    let sum = u + v + w;
    if sum <= 0.0 || !sum.is_finite() {
        return Err(CollisionError::NonConvergent);
    }
    Ok([u / sum, v / sum, w / sum])
}

fn weighted_point(weights: [f64; 3], a: Vec3, b: Vec3, c: Vec3) -> Result<Vec3, CollisionError> {
    let point = a * weights[0] + b * weights[1] + c * weights[2];
    point
        .is_finite()
        .then_some(point)
        .ok_or(CollisionError::ArithmeticOverflow)
}

fn scale_tolerance(value: Vec3) -> f64 {
    value.abs().max_element() * f64::EPSILON * 64.0
}

#[cfg(test)]
#[path = "epa_tests.rs"]
mod tests;
