use glam::{Mat3, Vec3};

use crate::shape::{Cuboid, Shape, Sphere};

pub mod error;
pub use error::MassError;

/// Mass, center of mass, and inertia tensor in body-local coordinates.
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

        let mass = checked_f32(mass)?;
        let inertia = Mat3::from_diagonal(Vec3::new(
            checked_f32(diagonal[0])?,
            checked_f32(diagonal[1])?,
            checked_f32(diagonal[2])?,
        ));
        Self::new(mass, Vec3::ZERO, inertia)
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

    let converted = value as f32;
    if value != 0.0 && converted == 0.0 {
        return Err(MassError::ArithmeticOverflow);
    }
    Ok(converted)
}

#[cfg(test)]
#[path = "mass_tests.rs"]
mod tests;
