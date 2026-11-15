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

fn scalar_godunov(left: f64, right: f64, mach: f64, gamma: f64) -> f64 {
    let lf = axial_flux(left, mach, gamma);
    let rf = axial_flux(right, mach, gamma);
    if left <= right {
        return lf.min(rf);
    }
    let sonic = (1.0 - mach.powi(2)) / (mach.powi(2) * (gamma + 1.0));
    if right <= sonic && sonic <= left {
        axial_flux(sonic, mach, gamma)
    } else {
        lf.max(rf)
    }
}

#[test]
fn scalar_godunov_conserves_but_selects_opposite_tsd_shock_orientation() {
    for u in [-0.1, 0.0, 0.1] {
        assert_eq!(scalar_godunov(u, u, 1.0, 1.4), axial_flux(u, 1.0, 1.4));
    }
    assert_eq!(
        scalar_godunov(0.1, -0.1, 1.0, 1.4),
        axial_flux(0.0, 1.0, 1.4)
    );
    assert_eq!(
        scalar_godunov(-0.1, 0.1, 1.0, 1.4),
        axial_flux(0.1, 1.0, 1.4)
    );
    for mach in [0.9_f64, 1.0, 1.1] {
        let sonic = (1.0 - mach.powi(2)) / (mach.powi(2) * 2.4);
        let (up, down) = (sonic + 0.02, sonic - 0.02);
        let physical = axial_flux(up, mach, 1.4);
        assert!((physical - axial_flux(down, mach, 1.4)).abs() < 1e-14);
        assert!(scalar_godunov(up, down, mach, 1.4) > physical);
        for n in [32, 64, 128] {
            for (left, right, stationary) in [(down, up, true), (up, down, false)] {
                let mut states = vec![left; n / 2 + 1];
                states.extend(vec![right; n / 2 + 1]);
                let flux: Vec<_> = states
                    .windows(2)
                    .map(|v| scalar_godunov(v[0], v[1], mach, 1.4))
                    .collect();
                let residual: Vec<_> = flux.windows(2).map(|v| v[1] - v[0]).collect();
                assert!(
                    (residual.iter().sum::<f64>() - (flux[flux.len() - 1] - flux[0])).abs() < 1e-14
                );
                let norm = residual.into_iter().map(f64::abs).fold(0.0_f64, f64::max);
                if stationary {
                    assert!(norm < 1e-14);
                } else {
                    assert!(norm > 0.0);
                }
            }
        }
    }
}

#[test]
fn murman_conservation_alone_admits_expansion_shocks() {
    // Jameson, Transonic Flow Calculations, section 2.1, equations (18)-(23).
    // This diagnostic does not claim entropy selection or a body solver.
    let faces = |slopes: &[f64]| -> Vec<f64> {
        slopes
            .windows(2)
            .map(|p| {
                let switch =
                    1.0 - 1.0_f64.powi(2) - 1.0_f64.powi(2) * 2.4 * (p[0] + p[1]) / 2.0 < 0.0;
                if switch {
                    axial_flux(p[0], 1.0, 1.4)
                } else {
                    axial_flux(p[1], 1.0, 1.4)
                }
            })
            .collect()
    };
    let f = faces(&[-0.1, -0.04, 0.08, 0.12, -0.03, -0.1]);
    let residual_sum: f64 = f.windows(2).map(|p| (p[1] - p[0]) / 0.1).sum();
    assert!((residual_sum * 0.1 - (f[f.len() - 1] - f[0])).abs() < 1e-14);
    for n in [32, 64, 128] {
        for (left, right) in [(0.1, -0.1), (-0.1, 0.1)] {
            let mut slopes = vec![left; n / 2];
            slopes.extend(vec![right; n / 2]);
            let f = faces(&slopes);
            let norm = f
                .windows(2)
                .map(|p| ((p[1] - p[0]) * n as f64).abs())
                .fold(0.0_f64, f64::max);
            assert!(norm < 1e-14);
        }
    }
}

#[test]
fn normal_shock_flux_factorization_and_type_change() {
    for mach in [0.9_f64, 1.0, 1.1] {
        let gamma = 1.4;
        let sonic = (1.0 - mach.powi(2)) / (mach.powi(2) * (gamma + 1.0));
        let (upstream, downstream) = (sonic + 0.02, sonic - 0.02);
        assert!(
            (axial_flux(upstream, mach, gamma) - axial_flux(downstream, mach, gamma)).abs() < 1e-14
        );
        let lambda = |u| 1.0 - mach.powi(2) - mach.powi(2) * (gamma + 1.0) * u;
        assert!(lambda(upstream) < 0.0 && lambda(downstream) > 0.0);
    }
    let (mach, gamma) = (0.95_f64, 1.4);
    for (u1, u2) in [(0.1, -0.05), (0.02, 0.07), (0.0, 0.0)] {
        let jump = axial_flux(u1, mach, gamma) - axial_flux(u2, mach, gamma);
        let factorized =
            (u1 - u2) * (1.0 - mach.powi(2) - mach.powi(2) * (gamma + 1.0) * (u1 + u2) / 2.0);
        assert!((jump - factorized).abs() < 1e-14);
    }
}

#[test]
fn sonic_flux_is_consistent_on_smooth_branches() {
    for u in [-0.1, 0.0, 0.1] {
        assert!((sonic_flux(u, u, 1.0, 1.4) - axial_flux(u, 1.0, 1.4)).abs() < 1e-14);
    }
    assert!((sonic_flux(0.1, 0.2, 1.0, 1.4) - axial_flux(0.1, 1.0, 1.4)).abs() < 1e-14);
    assert!((sonic_flux(-0.1, -0.2, 1.0, 1.4) - axial_flux(-0.2, 1.0, 1.4)).abs() < 1e-14);
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
    fn sample(&self, x: f64, radius: f64) -> Option<f64> {
        let xi = x / self.dx;
        let ri = radius / self.dr - 0.5;
        if !xi.is_finite() || !ri.is_finite() || xi < 0.0 || ri < 0.0 {
            return None;
        }
        let (i, k) = (xi.floor() as usize, ri.floor() as usize);
        if i + 1 >= self.phi.len() || k + 1 >= self.phi[0].len() {
            return None;
        }
        let (tx, tr) = (xi - i as f64, ri - k as f64);
        Some(
            (1.0 - tx) * ((1.0 - tr) * self.phi[i][k] + tr * self.phi[i][k + 1])
                + tx * ((1.0 - tr) * self.phi[i + 1][k] + tr * self.phi[i + 1][k + 1]),
        )
    }

    fn iterate(&mut self, tolerance: f64, max_iterations: usize) -> Result<(usize, f64), f64> {
        let nx = self.phi.len();
        let nr = self.phi[0].len();
        for iteration in 0..=max_iterations {
            let mut norm: f64 = 0.0;
            for i in 2..nx - 1 {
                for k in 0..nr - 1 {
                    let residual = self.residual(i, k);
                    if !residual.is_finite() {
                        return Err(residual);
                    }
                    norm = norm.max(residual.abs());
                }
            }
            if norm <= tolerance {
                return Ok((iteration, norm));
            }
            if iteration == max_iterations {
                return Err(norm);
            }
            for i in 2..nx - 1 {
                for k in 0..nr - 1 {
                    let old = self.phi[i][k];
                    let residual = self.residual(i, k);
                    let diagonal = self.diagonal(i, k);
                    if !diagonal.is_finite() || diagonal.abs() < 1e-14 {
                        return Err(diagonal);
                    }
                    let mut step = -residual / diagonal;
                    let mut accepted = false;
                    for _ in 0..20 {
                        self.phi[i][k] = old + step;
                        let trial = self.residual(i, k);
                        if trial.is_finite() && trial.abs() <= residual.abs() {
                            accepted = true;
                            break;
                        }
                        step *= 0.5;
                    }
                    if !accepted {
                        self.phi[i][k] = old;
                    }
                }
            }
        }
        unreachable!()
    }

    fn surface_cp(&self, i: usize, radius: f64, slope: f64) -> Option<f64> {
        if i == 0 || i + 1 >= self.phi.len() || !radius.is_finite() || !slope.is_finite() {
            return None;
        }
        let nr = self.phi[0].len();
        let index = radius / self.dr - 0.5;
        if !(0.0..=(nr - 1) as f64).contains(&index) {
            return None;
        }
        let k = (index.floor() as usize).min(nr - 2);
        let weight = index - k as f64;
        let low = (self.phi[i + 1][k] - self.phi[i - 1][k]) / (2.0 * self.dx);
        let high = (self.phi[i + 1][k + 1] - self.phi[i - 1][k + 1]) / (2.0 * self.dx);
        let cp = -2.0 * ((1.0 - weight) * low + weight * high) - slope.powi(2);
        cp.is_finite().then_some(cp)
    }

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
fn fixed_coordinate_interpolation_matches_linear_field() {
    for n in [9, 17, 33] {
        let h = 1.0 / (n - 1) as f64;
        let grid = MixedGrid {
            phi: (0..n)
                .map(|i| {
                    (0..n)
                        .map(|k| 2.0 * i as f64 * h + 3.0 * (k as f64 + 0.5) * h)
                        .collect()
                })
                .collect(),
            axis_flux: vec![0.0; n],
            dx: h,
            dr: h,
            mach: 0.9,
            gamma: 1.4,
        };
        assert!((grid.sample(0.5, 0.125).unwrap() - 1.375).abs() < 1e-14);
        assert!(grid.sample(0.1, 0.0).is_none());
    }
}

#[test]
fn fixed_coordinate_synthetic_diagnostic_matches_python() {
    use std::f64::consts::PI;
    // The actual outer radius differs across resolutions, as in the original
    // cell-centred Python diagnostic; this is not an experimental comparison.
    let cases = [
        (17, 0.75, 9.16663017e-6, 0.0841829837),
        (33, 0.75, -4.85319717e-5, 0.0878934802),
        (17, 1.25, 0.000140893568, 0.0882105903),
        (33, 1.25, -2.36133923e-6, 0.0922546393),
    ];
    for (n, outer, expected_phi, expected_velocity) in cases {
        let h = 1.0 / (n - 1) as f64;
        let nr = (outer / h + 0.5_f64).round_ties_even() as usize;
        let mut source: Vec<_> = (0..n)
            .map(|i| 0.01 * (2.0 * PI * i as f64 / (n - 1) as f64).sin())
            .collect();
        source[0] = 0.0;
        source[n - 1] = 0.0;
        let mut grid = MixedGrid {
            phi: vec![vec![0.0; nr]; n],
            axis_flux: source,
            dx: h,
            dr: h,
            mach: 0.9,
            gamma: 1.4,
        };
        assert!(grid.iterate(1e-6, 5000).unwrap().1 <= 1e-6);
        let phi = grid.sample(0.5, 0.125).unwrap();
        let velocity = (grid.sample(0.5 + h, 0.125).unwrap()
            - grid.sample(0.5 - h, 0.125).unwrap())
            / (2.0 * h);
        assert!(
            (phi - expected_phi).abs() < 1e-9,
            "n={n} outer={outer} phi={phi}"
        );
        assert!(
            (velocity - expected_velocity).abs() < 1e-8,
            "n={n} outer={outer} u={velocity}"
        );
    }
}

#[test]
fn harris_iteration_reproduces_python_diagnostic() {
    let (n, outer) = (17, 0.75);
    let dx = 1.0 / (n - 1) as f64;
    let nr = (outer / dx + 0.5_f64).round_ties_even() as usize;
    let dr = outer / (nr as f64 - 0.5);
    let mut grid = MixedGrid {
        phi: vec![vec![0.0; nr]; n],
        axis_flux: (0..n).map(|i| harris_geometry(i as f64 * dx).2).collect(),
        dx,
        dr,
        mach: 0.9,
        gamma: 1.4,
    };
    assert!(grid.iterate(1e-12, 1).is_err());
    // Restart from the same zero field used by the Python diagnostic.
    grid.phi = vec![vec![0.0; nr]; n];
    let (iterations, residual) = grid.iterate(1e-6, 10000).unwrap();
    let (radius, slope, _) = harris_geometry(0.5);
    let cp = grid.surface_cp((n - 1) / 2, radius, slope).unwrap();
    assert_eq!(iterations, 226);
    assert!(residual <= 1e-6);
    assert!((cp - (-0.12484869)).abs() < 1e-8, "Cp={cp}");
    assert_eq!(grid.surface_cp(8, 0.0, 0.0), None);
}

#[test]
fn synthetic_mixed_iteration_without_prescribed_shock() {
    use std::f64::consts::PI;
    for n in [9, 17, 25] {
        let h = 1.0 / (n - 1) as f64;
        let mut grid = MixedGrid {
            phi: vec![vec![0.0; n]; n],
            axis_flux: (0..n)
                .map(|i| 0.01 * (2.0 * PI * i as f64 / (n - 1) as f64).sin())
                .collect(),
            dx: h,
            dr: h,
            mach: 0.9,
            gamma: 1.4,
        };
        let (_, residual) = grid.iterate(1e-6, 3000).unwrap();
        assert!(residual <= 1e-6);
        let sonic = (1.0 - grid.mach.powi(2)) / (grid.mach.powi(2) * (grid.gamma + 1.0));
        let velocity: Vec<_> = (2..n - 1)
            .map(|i| (grid.phi[i + 1][1] - grid.phi[i - 1][1]) / (2.0 * h))
            .collect();
        let crossings: Vec<_> = velocity
            .windows(2)
            .enumerate()
            .filter(|(_, pair)| pair[0] > sonic && pair[1] <= sonic)
            .map(|(i, _)| (i + 2) as f64 * h)
            .collect();
        match n {
            9 => assert!(crossings.is_empty()),
            17 => assert_eq!(crossings, vec![0.5]),
            25 => {
                assert_eq!(crossings.len(), 1);
                assert!((crossings[0] - 14.0 / 24.0).abs() < 1e-14);
            }
            _ => unreachable!(),
        }
    }
}

#[test]
fn mixed_iteration_handles_subsonic_and_imposed_shock_profiles() {
    use std::f64::consts::PI;
    let n = 17;
    let h = 1.0 / (n - 1) as f64;
    let mut subsonic = MixedGrid {
        phi: vec![vec![0.0; 9]; n],
        axis_flux: (0..n)
            .map(|i| 0.0001 * (2.0 * PI * i as f64 / (n - 1) as f64).sin())
            .collect(),
        dx: h,
        dr: h,
        mach: 0.7,
        gamma: 1.4,
    };
    assert!(subsonic.iterate(1e-7, 3000).unwrap().1 <= 1e-7);
    // Outer boundary imposes this profile; it does not predict shock location.
    for n in [17, 33, 65] {
        let h = 1.0 / (n - 1) as f64;
        let mut grid = MixedGrid {
            phi: (0..n)
                .map(|i| {
                    let x = i as f64 * h;
                    vec![0.02 * x.min(0.5) - 0.02 * (x - 0.5).max(0.0); 4]
                })
                .collect(),
            axis_flux: vec![0.0; n],
            dx: h,
            dr: h,
            mach: 1.0,
            gamma: 1.4,
        };
        assert!(grid.iterate(1e-8, 3000).unwrap().1 <= 1e-8);
    }
}

#[test]
fn harris_grid_profiles_and_fixed_radius_field_match_python() {
    // The sting and boundaries do not reproduce the experiment. These are
    // numerical regression values, not measurements or validation tolerances.
    let cases = [
        (
            17,
            0.75,
            [
                0.196190, -0.037680, -0.105760, -0.124849, -0.111550, -0.059938, 0.066606,
            ],
            0.044320,
        ),
        (
            33,
            0.75,
            [
                0.104276, -0.051816, -0.107763, -0.123622, -0.109791, -0.058739, 0.075109,
            ],
            0.044704,
        ),
        (
            65,
            0.75,
            [
                0.077474, -0.056453, -0.108625, -0.123408, -0.109210, -0.057948, 0.081071,
            ],
            0.044871,
        ),
        (
            17,
            1.25,
            [
                0.198532, -0.037238, -0.107824, -0.128443, -0.113579, -0.058834, 0.069891,
            ],
            0.046207,
        ),
        (
            33,
            1.25,
            [
                0.106549, -0.051079, -0.110076, -0.127682, -0.112156, -0.057811, 0.078009,
            ],
            0.046782,
        ),
        (
            65,
            1.25,
            [
                0.080083, -0.055647, -0.111106, -0.127769, -0.111749, -0.057121, 0.083856,
            ],
            0.047047,
        ),
    ];
    for (n, outer, expected, expected_peak) in cases {
        let dx = 1.0 / (n - 1) as f64;
        let nr = (outer / dx + 0.5_f64).round_ties_even() as usize;
        let dr = outer / (nr as f64 - 0.5);
        let mut grid = MixedGrid {
            phi: vec![vec![0.0; nr]; n],
            axis_flux: (0..n).map(|i| harris_geometry(i as f64 * dx).2).collect(),
            dx,
            dr,
            mach: 0.9,
            gamma: 1.4,
        };
        let (_, residual) = grid.iterate(1e-6, 10000).unwrap();
        assert!(residual <= 1e-6);
        for (x, reference) in [0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875]
            .into_iter()
            .zip(expected)
        {
            let (radius, slope, _) = harris_geometry(x);
            let cp = grid
                .surface_cp((x / dx).round() as usize, radius, slope)
                .unwrap();
            assert!(
                (cp - reference).abs() < 1e-6,
                "n={n} outer={outer} x={x}: {cp}"
            );
        }
        // Table III(a), nominal A_base/A_max=0, Mach 0.90. The theoretical
        // 38.240-inch length normalizes x, not the truncated 36-inch model.
        // Interpolate Cp between node locations; neither interpolation nor
        // the printed measurement precision establishes uncertainty bounds.
        for (x, measured) in [(0.497, -0.135), (0.681, -0.094), (0.785, -0.025)] {
            let i = (x / dx).floor() as usize;
            let node_cp = |j: usize| {
                let (radius, slope, _) = harris_geometry(j as f64 * dx);
                grid.surface_cp(j, radius, slope).unwrap()
            };
            let t = x / dx - i as f64;
            let calculated = (1.0 - t) * node_cp(i) + t * node_cp(i + 1);
            println!(
                "n={n} outer={outer} x/l={x:.3} Cp_calc={calculated:.6} Cp_measured={measured:.3} difference={:+.6}",
                calculated - measured
            );
            assert!(calculated.is_finite());
        }
        let index = 0.125 / dr - 0.5;
        let k = index.floor() as usize;
        let w = index - k as f64;
        let samples: Vec<_> = grid
            .phi
            .iter()
            .map(|row| (1.0 - w) * row[k] + w * row[k + 1])
            .collect();
        let slopes: Vec<_> = samples.windows(2).map(|v| (v[1] - v[0]) / dx).collect();
        let peak = slopes.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        assert!((peak - expected_peak).abs() < 1e-6);
        let sonic = (1.0 - grid.mach.powi(2)) / (grid.mach.powi(2) * (grid.gamma + 1.0));
        assert!(slopes.windows(2).all(|u| !(u[0] > sonic && u[1] <= sonic)));
    }
}

#[test]
fn surface_pressure_uses_fixed_radius_derivative() {
    let h = 0.1;
    let grid = MixedGrid {
        phi: (0..9)
            .map(|i| {
                (0..8)
                    .map(|k| i as f64 * h * (k as f64 + 0.5) * h)
                    .collect()
            })
            .collect(),
        axis_flux: vec![0.0; 9],
        dx: h,
        dr: h,
        mach: 0.9,
        gamma: 1.4,
    };
    assert!((grid.surface_cp(4, 0.25, 0.2).unwrap() + 0.54).abs() < 1e-14);
}

#[test]
fn surface_pressure_converges_on_smooth_field_and_rejects_invalid_values() {
    let mut errors = Vec::new();
    for n in [16, 32, 64] {
        let h = 1.0 / n as f64;
        let mut grid = MixedGrid {
            phi: (0..=n)
                .map(|i| {
                    (0..n)
                        .map(|k| (i as f64 * h).sin() * ((k as f64 + 0.5) * h).exp())
                        .collect()
                })
                .collect(),
            axis_flux: vec![0.0; n + 1],
            dx: h,
            dr: h,
            mach: 0.9,
            gamma: 1.4,
        };
        let expected = -2.0 * 0.5_f64.cos() * 0.25_f64.exp() - 0.01;
        let cp = grid.surface_cp(n / 2, 0.25, 0.1).unwrap();
        errors.push((cp - expected).abs());
        assert_eq!(grid.surface_cp(n / 2, 0.0, 0.1), None);
        assert_eq!(grid.surface_cp(n / 2, f64::NAN, 0.1), None);
        grid.phi[n / 2 + 1][n / 4 - 1] = f64::INFINITY;
        assert_eq!(grid.surface_cp(n / 2, 0.25, 0.1), None);
    }
    assert!(errors[0] / errors[1] > 3.8);
    assert!(errors[1] / errors[2] > 3.8);
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
fn mixed_residual_distinguishes_prescribed_shock_orientations() {
    for n in [32, 64, 128] {
        let h = 1.0 / n as f64;
        for (up, down, compression) in [(0.02, -0.02, true), (-0.02, 0.02, false)] {
            let mut slopes = vec![up; n / 2];
            if compression {
                slopes.push(0.0);
            }
            slopes.extend(vec![down; n / 2]);
            let mut values = vec![0.0];
            for slope in slopes {
                values.push(values.last().unwrap() + h * slope);
            }
            let nx = values.len();
            let grid = MixedGrid {
                phi: values.into_iter().map(|v| vec![v; 3]).collect(),
                axis_flux: vec![0.0; nx],
                dx: h,
                dr: h,
                mach: 1.0,
                gamma: 1.4,
            };
            let norm = (2..nx - 1)
                .map(|i| grid.residual(i, 1).abs())
                .fold(0.0_f64, f64::max);
            if compression {
                assert!(norm < 1e-12);
            } else {
                assert!(norm > 1e-4);
            }
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
