//! Research diagnostics only: these do not validate a transonic body solver.
//! Geometry: Harris and Landrum, https://ntrs.nasa.gov/api/citations/19660003947/downloads/19660003947.pdf
//! TSD flux: Bailey, https://ntrs.nasa.gov/api/citations/19720004250/downloads/19720004250.pdf

fn harris_geometry(x: f64) -> (f64, f64, f64) {
    const CUTOFF: f64 = 36.0 / 38.240;
    const FINENESS: f64 = 7.0;
    let z = x.min(CUTOFF);
    let shape = (1.0 - (2.0 * z - 1.0).powi(2)).max(0.0);
    let radius = shape.powf(0.75) / (2.0 * FINENESS);
    if x >= CUTOFF || z <= 0.0 {
        return (radius, 0.0, 0.0);
    }
    let slope = -3.0 * (2.0 * z - 1.0) * shape.powf(-0.25) / (2.0 * FINENESS);
    (radius, slope, radius * slope)
}

fn axial_flux(u: f64, mach: f64, gamma: f64) -> f64 {
    (1.0 - mach * mach) * u - 0.5 * mach * mach * (gamma + 1.0) * u * u
}

fn sonic_flux(left: f64, right: f64, mach: f64, gamma: f64) -> f64 {
    let sonic = (1.0 - mach * mach) / (mach * mach * (gamma + 1.0));
    let peak = axial_flux(sonic, mach, gamma);
    let upstream_loss = if left > sonic {
        peak - axial_flux(left, mach, gamma)
    } else {
        0.0
    };
    let downstream_loss = if right < sonic {
        peak - axial_flux(right, mach, gamma)
    } else {
        0.0
    };
    peak - upstream_loss - downstream_loss
}

struct MixedGrid {
    phi: Vec<Vec<f64>>,
    axis_flux: Vec<f64>,
    dx: f64,
    dr: f64,
    mach: f64,
    gamma: f64,
}

impl MixedGrid {
    fn residual(&self, i: usize, k: usize) -> f64 {
        let u0 = (self.phi[i - 1][k] - self.phi[i - 2][k]) / self.dx;
        let u1 = (self.phi[i][k] - self.phi[i - 1][k]) / self.dx;
        let u2 = (self.phi[i + 1][k] - self.phi[i][k]) / self.dx;
        let axial = (sonic_flux(u1, u2, self.mach, self.gamma)
            - sonic_flux(u0, u1, self.mach, self.gamma))
            / self.dx;
        let r = (k as f64 + 0.5) * self.dr;
        let lower = k as f64 * self.dr / (r * self.dr.powi(2));
        let upper = (k + 1) as f64 * self.dr / (r * self.dr.powi(2));
        let p = self.phi[i][k];
        let radial = if k == 0 {
            -self.axis_flux[i] / (r * self.dr)
        } else {
            lower * (self.phi[i][k - 1] - p)
        };
        axial + radial + upper * (self.phi[i][k + 1] - p)
    }

    fn diagonal(&self, i: usize, k: usize) -> f64 {
        let coefficient =
            |u: f64| 1.0 - self.mach.powi(2) - self.mach.powi(2) * (self.gamma + 1.0) * u;
        let u = (self.phi[i][k] - self.phi[i - 1][k]) / self.dx;
        let forward = (self.phi[i + 1][k] - self.phi[i][k]) / self.dx;
        -(coefficient(u).abs() + coefficient(forward).max(0.0)) / self.dx.powi(2)
            - (2 * k + 1) as f64 / ((k as f64 + 0.5) * self.dr.powi(2))
    }
}

#[test]
fn mixed_residual_diagonal_matches_finite_difference() {
    let (dx, mach, gamma) = (0.1, 0.9_f64, 1.4);
    let sonic = (1.0 - mach.powi(2)) / (mach.powi(2) * (gamma + 1.0));
    for u in [sonic - 0.1, sonic, sonic + 0.1] {
        for k in 0..2 {
            let mut grid = MixedGrid {
                phi: vec![vec![0.0; 3]; 5],
                axis_flux: vec![0.0; 5],
                dx,
                dr: dx,
                mach,
                gamma,
            };
            grid.phi[2][k] = dx * u;
            let eps = 1e-8;
            grid.phi[2][k] += eps;
            let plus = grid.residual(2, k);
            grid.phi[2][k] -= 2.0 * eps;
            let minus = grid.residual(2, k);
            grid.phi[2][k] += eps;
            assert!(((plus - minus) / (2.0 * eps) - grid.diagonal(2, k)).abs() < 2e-5);
        }
    }
}

#[test]
fn mixed_residual_matches_smooth_subsonic_quadratic() {
    let (n, a, b, mach, gamma) = (32, 0.005, 0.007, 0.7_f64, 1.4);
    let h = 1.0 / n as f64;
    let grid = MixedGrid {
        phi: (0..=n)
            .map(|i| {
                (0..n)
                    .map(|k| a * (i as f64 * h).powi(2) + b * ((k as f64 + 0.5) * h).powi(2))
                    .collect()
            })
            .collect(),
        axis_flux: vec![0.0; n + 1],
        dx: h,
        dr: h,
        mach,
        gamma,
    };
    let i = n / 2;
    let expected =
        (1.0 - mach.powi(2) - mach.powi(2) * (gamma + 1.0) * 2.0 * a * i as f64 * h) * 2.0 * a
            + 4.0 * b;
    assert!((grid.residual(i, n / 4) - expected).abs() < 1e-11);
}

#[test]
fn harris_geometry_and_source_match_derivative() {
    let (radius, slope, flux) = harris_geometry(0.5);
    assert!((radius - 1.0 / 14.0).abs() < 1e-14);
    assert_eq!((slope, flux), (0.0, 0.0));
    for x in [0.25, 0.75, 0.9] {
        let (radius, slope, flux) = harris_geometry(x);
        let eps = 1e-6;
        let derivative = (harris_geometry(x + eps).0 - harris_geometry(x - eps).0) / (2.0 * eps);
        assert!((slope - derivative).abs() < 1e-8);
        assert!((flux - radius * derivative).abs() < 1e-8);
    }
    assert_eq!(harris_geometry(0.95).0, harris_geometry(1.0).0);
    assert_eq!(harris_geometry(0.95).2, 0.0);
}

#[test]
fn sonic_flux_distinguishes_prescribed_compression_and_expansion() {
    for mach in [0.9, 1.0, 1.1] {
        let gamma = 1.4;
        let sonic = (1.0 - mach * mach) / (mach * mach * (gamma + 1.0));
        let (up, down) = (sonic + 0.02, sonic - 0.02);
        let expected = axial_flux(up, mach, gamma);
        assert!((axial_flux(down, mach, gamma) - expected).abs() < 1e-14);
        assert!((sonic_flux(up, sonic, mach, gamma) - expected).abs() < 1e-14);
        assert!((sonic_flux(sonic, down, mach, gamma) - expected).abs() < 1e-14);
        assert!((sonic_flux(down, up, mach, gamma) - expected).abs() > 1e-6);
    }
}
