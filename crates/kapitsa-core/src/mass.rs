use glam::{DMat3, DVec3, Mat3, Vec3};

use crate::{
    shape::{Cuboid, Shape, Sphere},
    transform::Transform,
};

pub mod error;
pub use error::MassError;

/// Mass in kilograms, center of mass in body-local meters, and inertia in kg·m².
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MassProperties {
    mass: f32,
    center_of_mass: Vec3,
    inertia: Mat3,
}

impl MassProperties {
    pub fn new(mass: f32, center_of_mass: Vec3, inertia: Mat3) -> Result<Self, MassError> {
        if !mass.is_finite() {
            return Err(MassError::NonFiniteMass);
        }
        if mass <= 0.0 {
            return Err(MassError::NonPositiveMass);
        }
        if !center_of_mass.is_finite() {
            return Err(MassError::NonFiniteCenterOfMass);
        }

        let columns = inertia.to_cols_array();
        if columns.iter().any(|value| !value.is_finite()) {
            return Err(MassError::NonFiniteInertia);
        }

        let matrix = [
            [
                f64::from(columns[0]),
                f64::from(columns[3]),
                f64::from(columns[6]),
            ],
            [
                f64::from(columns[1]),
                f64::from(columns[4]),
                f64::from(columns[7]),
            ],
            [
                f64::from(columns[2]),
                f64::from(columns[5]),
                f64::from(columns[8]),
            ],
        ];
        for (row, column) in [(0, 1), (0, 2), (1, 2)] {
            if !approximately_equal(matrix[row][column], matrix[column][row]) {
                return Err(MassError::NonSymmetricInertia);
            }
        }

        let xx = matrix[0][0];
        let xy = (matrix[0][1] + matrix[1][0]) * 0.5;
        let xz = (matrix[0][2] + matrix[2][0]) * 0.5;
        let yy = matrix[1][1];
        let yz = (matrix[1][2] + matrix[2][1]) * 0.5;
        let zz = matrix[2][2];
        if xx <= 0.0 {
            return Err(MassError::NonPositiveDefiniteInertia);
        }

        let second_pivot = yy - xy * xy / xx;
        if !second_pivot.is_finite() || second_pivot <= 0.0 {
            return Err(MassError::NonPositiveDefiniteInertia);
        }

        let third_pivot = zz - xz * xz / xx - (yz - xy * xz / xx).powi(2) / second_pivot;
        if !third_pivot.is_finite() || third_pivot <= 0.0 {
            return Err(MassError::NonPositiveDefiniteInertia);
        }

        Ok(Self {
            mass,
            center_of_mass,
            inertia,
        })
    }

    pub fn mass(&self) -> f32 {
        self.mass
    }

    pub fn center_of_mass(&self) -> Vec3 {
        self.center_of_mass
    }

    pub fn inertia(&self) -> Mat3 {
        self.inertia
    }

    pub fn from_shape(shape: &Shape, density: f32) -> Result<Self, MassError> {
        if !density.is_finite() {
            return Err(MassError::NonFiniteDensity);
        }
        if density <= 0.0 {
            return Err(MassError::NonPositiveDensity);
        }

        let (mass, diagonal) = match shape {
            Shape::Sphere(sphere) => sphere_mass_properties(sphere, f64::from(density)),
            Shape::Cuboid(cuboid) => cuboid_mass_properties(cuboid, f64::from(density)),
            _ => return Err(MassError::UnsupportedShape),
        };

        let mass = checked_positive_f32(mass)?;
        let inertia = Mat3::from_diagonal(Vec3::new(
            checked_positive_f32(diagonal[0])?,
            checked_positive_f32(diagonal[1])?,
            checked_positive_f32(diagonal[2])?,
        ));
        Self::new(mass, Vec3::ZERO, inertia)
    }

    /// Adds a part after transforming it into this body's local coordinate system.
    pub fn push(&mut self, part: &Self, transform: &Transform) -> Result<(), MassError> {
        let rotation = rotation_matrix(transform);
        let part_center =
            rotation * to_dvec3(part.center_of_mass) + to_dvec3(transform.translation());
        let part_inertia = rotation * to_dmat3(part.inertia) * rotation.transpose();
        let self_center = to_dvec3(self.center_of_mass);
        let self_mass = f64::from(self.mass);
        let part_mass = f64::from(part.mass);
        let total_mass = self_mass + part_mass;
        if !total_mass.is_finite() || total_mass <= 0.0 || !part_center.is_finite() {
            return Err(MassError::ArithmeticOverflow);
        }

        let center = (self_center * self_mass + part_center * part_mass) / total_mass;
        if !center.is_finite() {
            return Err(MassError::ArithmeticOverflow);
        }

        let self_offset = self_center - center;
        let part_offset = part_center - center;
        let inertia = to_dmat3(self.inertia)
            + parallel_axis(self_mass, self_offset)
            + part_inertia
            + parallel_axis(part_mass, part_offset);
        let next = Self::new(
            checked_positive_f32(total_mass)?,
            to_vec3(center)?,
            to_mat3(inertia)?,
        )?;

        *self = next;
        Ok(())
    }

    /// Combines parts transformed into one common coordinate system.
    pub fn combine(parts: &[(Self, Transform)]) -> Result<Self, MassError> {
        let Some((first, transform)) = parts.first() else {
            return Err(MassError::EmptyComposition);
        };

        let mut combined = transformed(first, transform)?;
        for (part, transform) in &parts[1..] {
            combined.push(part, transform)?;
        }
        Ok(combined)
    }

    /// Replaces these properties with the combination of the supplied parts.
    pub fn set_combined(&mut self, parts: &[(Self, Transform)]) -> Result<(), MassError> {
        let combined = Self::combine(parts)?;
        *self = combined;
        Ok(())
    }
}

fn approximately_equal(first: f64, second: f64) -> bool {
    let scale = first.abs().max(second.abs());
    (first - second).abs() <= 8.0 * f64::from(f32::EPSILON) * scale
}

fn sphere_mass_properties(sphere: &Sphere, density: f64) -> (f64, [f64; 3]) {
    let radius = f64::from(sphere.radius());
    let mass = density * (4.0 / 3.0) * std::f64::consts::PI * radius.powi(3);
    let inertia = (2.0 / 5.0) * mass * radius.powi(2);
    (mass, [inertia; 3])
}

fn cuboid_mass_properties(cuboid: &Cuboid, density: f64) -> (f64, [f64; 3]) {
    let size = cuboid.size();
    let x = f64::from(size.x);
    let y = f64::from(size.y);
    let z = f64::from(size.z);
    let mass = density * x * y * z;
    let divisor = 12.0;
    let inertia = [
        mass * (y.powi(2) + z.powi(2)) / divisor,
        mass * (x.powi(2) + z.powi(2)) / divisor,
        mass * (x.powi(2) + y.powi(2)) / divisor,
    ];
    (mass, inertia)
}

fn checked_f32(value: f64) -> Result<f32, MassError> {
    if !value.is_finite() || value.abs() > f64::from(f32::MAX) {
        return Err(MassError::ArithmeticOverflow);
    }

    Ok(value as f32)
}

fn checked_positive_f32(value: f64) -> Result<f32, MassError> {
    let converted = checked_f32(value)?;
    if value <= 0.0 || converted <= 0.0 {
        return Err(MassError::ArithmeticOverflow);
    }
    Ok(converted)
}

fn to_vec3(value: DVec3) -> Result<Vec3, MassError> {
    Ok(Vec3::new(
        checked_f32(value.x)?,
        checked_f32(value.y)?,
        checked_f32(value.z)?,
    ))
}

fn to_mat3(value: DMat3) -> Result<Mat3, MassError> {
    let columns = value.to_cols_array();
    let mut converted = [0.0; 9];
    for (output, component) in converted.iter_mut().zip(columns) {
        *output = checked_f32(component)?;
    }
    Ok(Mat3::from_cols_array(&converted))
}

fn to_dvec3(value: Vec3) -> DVec3 {
    DVec3::new(f64::from(value.x), f64::from(value.y), f64::from(value.z))
}

fn to_dmat3(value: Mat3) -> DMat3 {
    let columns = value.to_cols_array();
    DMat3::from_cols(
        DVec3::new(
            f64::from(columns[0]),
            f64::from(columns[1]),
            f64::from(columns[2]),
        ),
        DVec3::new(
            f64::from(columns[3]),
            f64::from(columns[4]),
            f64::from(columns[5]),
        ),
        DVec3::new(
            f64::from(columns[6]),
            f64::from(columns[7]),
            f64::from(columns[8]),
        ),
    )
}

fn rotation_matrix(transform: &Transform) -> DMat3 {
    let rotation = transform.rotation();
    let length = f64::from(rotation.length());
    let x = f64::from(rotation.x) / length;
    let y = f64::from(rotation.y) / length;
    let z = f64::from(rotation.z) / length;
    let w = f64::from(rotation.w) / length;
    let x2 = x * x;
    let y2 = y * y;
    let z2 = z * z;
    let w2 = w * w;

    DMat3::from_cols(
        DVec3::new(
            w2 + x2 - y2 - z2,
            2.0 * (x * y + w * z),
            2.0 * (x * z - w * y),
        ),
        DVec3::new(
            2.0 * (x * y - w * z),
            w2 - x2 + y2 - z2,
            2.0 * (y * z + w * x),
        ),
        DVec3::new(
            2.0 * (x * z + w * y),
            2.0 * (y * z - w * x),
            w2 - x2 - y2 + z2,
        ),
    )
}

fn parallel_axis(mass: f64, offset: DVec3) -> DMat3 {
    let distance_squared = offset.length_squared();
    let outer_product = DMat3::from_cols(offset * offset.x, offset * offset.y, offset * offset.z);
    (DMat3::IDENTITY * distance_squared - outer_product) * mass
}

fn transformed(part: &MassProperties, transform: &Transform) -> Result<MassProperties, MassError> {
    let rotation = rotation_matrix(transform);
    let center = rotation * to_dvec3(part.center_of_mass) + to_dvec3(transform.translation());
    let inertia = rotation * to_dmat3(part.inertia) * rotation.transpose();
    MassProperties::new(part.mass, to_vec3(center)?, to_mat3(inertia)?)
}

#[cfg(test)]
#[path = "mass_tests.rs"]
mod tests;
