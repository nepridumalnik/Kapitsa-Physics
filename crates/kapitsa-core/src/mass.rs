use kapitsa_math::{CholeskyError, Mat3, Vec3};

use crate::{
    shape::{Cuboid, Shape, Sphere},
    transform::Transform,
};

pub mod error;
pub use error::MassError;

/// Mass in kilograms, center of mass in body-local meters, and inertia in kg·m².
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MassProperties {
    mass: f64,
    center_of_mass: Vec3,
    inertia: Mat3,
}

impl MassProperties {
    pub fn new(mass: f64, center_of_mass: Vec3, inertia: Mat3) -> Result<Self, MassError> {
        if !mass.is_finite() {
            return Err(MassError::NonFiniteMass);
        }
        if mass <= 0.0 {
            return Err(MassError::NonPositiveMass);
        }
        if !center_of_mass.is_finite() {
            return Err(MassError::NonFiniteCenterOfMass);
        }

        let inertia = symmetrize_inertia(inertia)?;
        if !is_physically_realizable(inertia) {
            return Err(MassError::NonPhysicalInertia);
        }

        Ok(Self {
            mass,
            center_of_mass,
            inertia,
        })
    }

    pub fn mass(&self) -> f64 {
        self.mass
    }

    pub fn center_of_mass(&self) -> Vec3 {
        self.center_of_mass
    }

    pub fn inertia(&self) -> Mat3 {
        self.inertia
    }

    pub fn from_shape(shape: &Shape, density: f64) -> Result<Self, MassError> {
        if !density.is_finite() {
            return Err(MassError::NonFiniteDensity);
        }
        if density <= 0.0 {
            return Err(MassError::NonPositiveDensity);
        }

        let (mass, diagonal) = match shape {
            Shape::Sphere(sphere) => sphere_mass_properties(sphere, density),
            Shape::Cuboid(cuboid) => cuboid_mass_properties(cuboid, density),
            _ => return Err(MassError::UnsupportedShape),
        };
        if !mass.is_finite()
            || mass <= 0.0
            || diagonal
                .iter()
                .any(|value| !value.is_finite() || *value <= 0.0)
        {
            return Err(MassError::ArithmeticOverflow);
        }

        let inertia = Mat3::from_diagonal(Vec3::new(diagonal[0], diagonal[1], diagonal[2]));
        Self::new(mass, Vec3::ZERO, inertia)
    }

    /// Adds a part after transforming it into this body's local coordinate system.
    pub fn push(&mut self, part: &Self, transform: &Transform) -> Result<(), MassError> {
        let rotation = rotation_matrix(transform)?;
        let part_center = rotation
            .checked_mul_vector(part.center_of_mass)
            .and_then(|center| center.checked_add(transform.translation()))
            .ok_or(MassError::ArithmeticOverflow)?;
        let part_inertia = rotation
            .checked_mul_matrix(part.inertia)
            .and_then(|rotated| rotated.checked_mul_matrix(rotation.transpose()))
            .ok_or(MassError::ArithmeticOverflow)?;
        let total_mass = self.mass + part.mass;
        if !total_mass.is_finite() || total_mass <= 0.0 {
            return Err(MassError::ArithmeticOverflow);
        }

        let center =
            self.center_of_mass * (self.mass / total_mass) + part_center * (part.mass / total_mass);
        if !center.is_finite() {
            return Err(MassError::ArithmeticOverflow);
        }

        let self_offset = self.center_of_mass - center;
        let part_offset = part_center - center;
        let inertia = self.inertia
            + parallel_axis(self.mass, self_offset)
            + part_inertia
            + parallel_axis(part.mass, part_offset);
        if !inertia.is_finite() {
            return Err(MassError::ArithmeticOverflow);
        }

        let next = Self::new(total_mass, center, inertia)?;
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

fn symmetrize_inertia(inertia: Mat3) -> Result<Mat3, MassError> {
    if !inertia.is_finite() {
        return Err(MassError::NonFiniteInertia);
    }

    let mut rows = inertia_rows(inertia);
    let scale = inertia
        .to_cols_array()
        .into_iter()
        .fold(0.0_f64, |scale, value| scale.max(value.abs()));
    let tolerance = 32.0 * f64::EPSILON * scale;
    for (row, column) in [(0, 1), (0, 2), (1, 2)] {
        if (rows[row][column] - rows[column][row]).abs() > tolerance {
            return Err(MassError::NonSymmetricInertia);
        }
        let average = rows[row][column] * 0.5 + rows[column][row] * 0.5;
        rows[row][column] = average;
        rows[column][row] = average;
    }

    let symmetric = Mat3::from_rows(rows);
    match symmetric.solve_spd(Vec3::ZERO) {
        Ok(_) => Ok(symmetric),
        Err(CholeskyError::NotPositiveDefinite) => Err(MassError::NonPositiveDefiniteInertia),
        Err(CholeskyError::NotSymmetric) => Err(MassError::NonSymmetricInertia),
        Err(CholeskyError::NonFiniteInput | CholeskyError::ResultOutOfRange) => {
            Err(MassError::ArithmeticOverflow)
        }
    }
}

fn is_physically_realizable(inertia: Mat3) -> bool {
    let scale = inertia
        .to_cols_array()
        .into_iter()
        .fold(0.0_f64, |scale, value| scale.max(value.abs()));
    if !scale.is_finite() || scale <= 0.0 {
        return false;
    }

    let normalized = inertia_rows(inertia).map(|row| row.map(|value| value / scale));
    let half_trace = (normalized[0][0] + normalized[1][1] + normalized[2][2]) * 0.5;
    let mut covariance = [
        [
            half_trace - normalized[0][0],
            -normalized[0][1],
            -normalized[0][2],
        ],
        [
            -normalized[1][0],
            half_trace - normalized[1][1],
            -normalized[1][2],
        ],
        [
            -normalized[2][0],
            -normalized[2][1],
            half_trace - normalized[2][2],
        ],
    ];
    let covariance_scale = covariance
        .iter()
        .flatten()
        .fold(0.0_f64, |scale, value| scale.max(value.abs()));
    if !covariance_scale.is_finite() || covariance_scale == 0.0 {
        return false;
    }
    for row in &mut covariance {
        for value in row {
            *value /= covariance_scale;
        }
    }

    let epsilon = f64::EPSILON;
    let diagonal_tolerance = 16.0 * epsilon;
    let second_order_tolerance = 32.0 * epsilon;
    let third_order_tolerance = 64.0 * epsilon;
    if covariance[0][0] < -diagonal_tolerance
        || covariance[1][1] < -diagonal_tolerance
        || covariance[2][2] < -diagonal_tolerance
    {
        return false;
    }

    let minor_01 = covariance[0][0] * covariance[1][1] - covariance[0][1].powi(2);
    let minor_02 = covariance[0][0] * covariance[2][2] - covariance[0][2].powi(2);
    let minor_12 = covariance[1][1] * covariance[2][2] - covariance[1][2].powi(2);
    if minor_01 < -second_order_tolerance
        || minor_02 < -second_order_tolerance
        || minor_12 < -second_order_tolerance
    {
        return false;
    }

    let determinant = covariance[0][0]
        * (covariance[1][1] * covariance[2][2] - covariance[1][2].powi(2))
        - covariance[0][1]
            * (covariance[0][1] * covariance[2][2] - covariance[1][2] * covariance[0][2])
        + covariance[0][2]
            * (covariance[0][1] * covariance[1][2] - covariance[1][1] * covariance[0][2]);
    determinant.is_finite() && determinant >= -third_order_tolerance
}

fn sphere_mass_properties(sphere: &Sphere, density: f64) -> (f64, [f64; 3]) {
    let radius = sphere.radius();
    let mass = density * (4.0 / 3.0) * std::f64::consts::PI * radius.powi(3);
    let inertia = (2.0 / 5.0) * mass * radius.powi(2);
    (mass, [inertia; 3])
}

fn cuboid_mass_properties(cuboid: &Cuboid, density: f64) -> (f64, [f64; 3]) {
    let size = cuboid.size();
    let x = size.x;
    let y = size.y;
    let z = size.z;
    let mass = density * x * y * z;
    let inertia = [
        mass * (y.powi(2) + z.powi(2)) / 12.0,
        mass * (x.powi(2) + z.powi(2)) / 12.0,
        mass * (x.powi(2) + y.powi(2)) / 12.0,
    ];
    (mass, inertia)
}

fn rotation_matrix(transform: &Transform) -> Result<Mat3, MassError> {
    transform
        .rotation()
        .to_mat3()
        .ok_or(MassError::ArithmeticOverflow)
}

fn inertia_rows(inertia: Mat3) -> [[f64; 3]; 3] {
    let columns = inertia.to_cols_array();
    [
        [columns[0], columns[3], columns[6]],
        [columns[1], columns[4], columns[7]],
        [columns[2], columns[5], columns[8]],
    ]
}

fn parallel_axis(mass: f64, offset: Vec3) -> Mat3 {
    let distance_squared = offset.length_squared();
    let outer_product = Mat3::from_cols(offset * offset.x, offset * offset.y, offset * offset.z);
    (Mat3::IDENTITY * distance_squared - outer_product) * mass
}

fn transformed(part: &MassProperties, transform: &Transform) -> Result<MassProperties, MassError> {
    let rotation = rotation_matrix(transform)?;
    let center = rotation
        .checked_mul_vector(part.center_of_mass)
        .and_then(|center| center.checked_add(transform.translation()))
        .ok_or(MassError::ArithmeticOverflow)?;
    let inertia = rotation
        .checked_mul_matrix(part.inertia)
        .and_then(|rotated| rotated.checked_mul_matrix(rotation.transpose()))
        .ok_or(MassError::ArithmeticOverflow)?;
    MassProperties::new(part.mass, center, inertia)
}

#[cfg(test)]
#[path = "mass_tests.rs"]
mod tests;
