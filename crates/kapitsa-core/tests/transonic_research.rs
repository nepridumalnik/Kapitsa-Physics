//! Research diagnostics only: these do not validate a transonic body solver.
//! Geometry: Harris and Landrum, https://ntrs.nasa.gov/api/citations/19660003947/downloads/19660003947.pdf
//! TSD flux: Bailey, https://ntrs.nasa.gov/api/citations/19720004250/downloads/19720004250.pdf

#[path = "support/harris_data.rs"]
mod harris_data;

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

#[test]
fn harris_fixed_upstream_row_omits_nonzero_axis_source() {
    // The mixed stencil starts at i=2, leaving i=1 fixed even though the
    // Harris body source at that station is nonzero. This diagnostic does not
    // establish a suitable upstream boundary condition or an error bound.
    for n in [17, 33, 65, 97] {
        let dx = 1.0 / (n - 1) as f64;
        let source = harris_geometry(dx).2;
        let integrated_source = source * dx;
        assert!(source.is_finite() && source > 0.0);
        println!(
            "n={n} fixed_row_x={dx:.6} omitted_axis_flux={source:.9} source_times_dx={integrated_source:.9}"
        );
    }
}

#[test]
fn harris_axis_source_matches_area_derivative_away_from_truncation() {
    // For the slender-body source R R' = (R²)'/2. At the cutoff the
    // cylindrical continuation is continuous in radius but not in slope;
    // the pointwise value there is the downstream one-sided convention.
    const CUTOFF: f64 = 36.0 / 38.240;
    const DIFFERENCE_STEP: f64 = 1e-6;
    const DERIVATIVE_TOLERANCE: f64 = 1e-8;
    for x in [0.1, 0.5, 0.9, 0.95] {
        let (radius, slope, source) = harris_geometry(x);
        let previous = harris_geometry(x - DIFFERENCE_STEP).0;
        let next = harris_geometry(x + DIFFERENCE_STEP).0;
        let area_derivative = (next * next - previous * previous) / (4.0 * DIFFERENCE_STEP);
        assert!((source - radius * slope).abs() < DERIVATIVE_TOLERANCE);
        assert!((source - area_derivative).abs() < DERIVATIVE_TOLERANCE);
    }
    let upstream = harris_geometry(CUTOFF - DIFFERENCE_STEP);
    let cutoff = harris_geometry(CUTOFF);
    let downstream = harris_geometry(CUTOFF + DIFFERENCE_STEP);
    assert!(upstream.2 < 0.0);
    assert_eq!(cutoff.1, 0.0);
    assert_eq!(cutoff.2, 0.0);
    assert_eq!(downstream.0, cutoff.0);
    assert_eq!(downstream.2, 0.0);
    println!(
        "cutoff={CUTOFF:.9} radius={:.9} upstream_source={:.9} downstream_source={:.9}",
        cutoff.0, upstream.2, downstream.2
    );
}

#[test]
fn radial_face_difference_converges_for_logarithmic_line_source() {
    // For locally constant q = R R', the axisymmetric line-source field
    // phi = q ln(r/R) has phi_r(R) = R'. This verifies reconstruction only;
    // axial variation and the nonlinear body boundary are not represented.
    const RADIUS: f64 = 0.05;
    const SLOPE: f64 = 0.2;
    const SOURCE: f64 = RADIUS * SLOPE;
    let mut previous_error = f64::INFINITY;
    for cells_per_radius in [4_usize, 8, 16, 32] {
        let dr = RADIUS / cells_per_radius as f64;
        let lower_radius = RADIUS - 0.5 * dr;
        let upper_radius = RADIUS + 0.5 * dr;
        let lower = SOURCE * (lower_radius / RADIUS).ln();
        let upper = SOURCE * (upper_radius / RADIUS).ln();
        let derivative = (upper - lower) / dr;
        let error = (derivative - SLOPE).abs();
        assert!(error < previous_error);
        println!("cells_per_radius={cells_per_radius} phi_r={derivative:.9} error={error:.9}");
        previous_error = error;
    }
}

#[test]
fn radial_difference_off_face_converges_at_surface() {
    // The diagnostic evaluates the difference between the two cell centres
    // bracketing the surface. That difference is centred on their shared
    // face, not on the surface; offset produces a first-order error.
    const RADIUS: f64 = 0.05;
    const SLOPE: f64 = 0.2;
    const SOURCE: f64 = RADIUS * SLOPE;
    const OFFSET_IN_CELLS: f64 = 0.25;
    let mut previous_error = f64::INFINITY;
    for cells_per_radius in [4_usize, 8, 16, 32] {
        let dr = RADIUS / cells_per_radius as f64;
        let face = RADIUS - OFFSET_IN_CELLS * dr;
        let lower = SOURCE * ((face - 0.5 * dr) / RADIUS).ln();
        let upper = SOURCE * ((face + 0.5 * dr) / RADIUS).ln();
        let derivative = (upper - lower) / dr;
        let surface_error = (derivative - SLOPE).abs();
        let face_error = (derivative - SOURCE / face).abs();
        assert!(surface_error < previous_error);
        assert!(face_error < surface_error);
        println!(
            "cells_per_radius={cells_per_radius} surface_error={surface_error:.9} face_error={face_error:.9}"
        );
        previous_error = surface_error;
    }
}

#[test]
#[ignore = "extended-domain research diagnostic; run explicitly in release mode"]
fn harris_extended_axial_domain_diagnostic() -> Result<(), String> {
    // Bailey TN D-6582, far-field boundaries: sonic/subsonic cases require
    // at least two body lengths fore and aft. This shorter finite domain is
    // only a sensitivity experiment, not that far-field limit or a tunnel.
    const EXTENSION: f64 = 0.5;
    const OUTER: f64 = 1.25;
    const TOLERANCE: f64 = 1e-6;
    const MAX_ITERATIONS: usize = 40000;
    for body_intervals in [16_usize, 32, 64] {
        let dx = 1.0 / body_intervals as f64;
        let nx = 2 * body_intervals + 1;
        let nr = (OUTER / dx + 0.5).round_ties_even() as usize;
        let dr = OUTER / (nr as f64 - 0.5);
        let source: Vec<_> = (0..nx)
            .map(|i| {
                let x = i as f64 * dx - EXTENSION;
                if (0.0..=1.0).contains(&x) {
                    harris_geometry(x).2
                } else {
                    0.0
                }
            })
            .collect();
        assert_eq!(source[0], 0.0);
        assert_eq!(source[1], 0.0);
        let mut grid = MixedGrid {
            phi: vec![vec![0.0; nr]; nx],
            axis_flux: source,
            dx,
            dr,
            mach: 1.0,
            gamma: 1.4,
        };
        let (iterations, residual) = grid
            .iterate(TOLERANCE, MAX_ITERATIONS)
            .map_err(|value| format!("body intervals {body_intervals}: residual {value}"))?;
        println!("body_intervals={body_intervals} iterations={iterations} residual={residual:.3e}");
        for &(x, measured) in harris_data::TABLE_III_A_MACH_100_AFT {
            let index = (x + EXTENSION) / dx;
            let i = index.floor() as usize;
            let node_cp = |j: usize| {
                let (radius, slope, _) = harris_geometry(j as f64 * dx - EXTENSION);
                grid.surface_cp(j, radius, slope)
            };
            let (Some(left), Some(right)) = (node_cp(i), node_cp(i + 1)) else {
                return Err(format!("unavailable pressure at x/l={x}"));
            };
            let t = index - i as f64;
            let cp = (1.0 - t) * left + t * right;
            println!(
                "x/l={x:.3} cp={cp:.6} measured={measured:.3} delta={:+.6}",
                cp - measured
            );
        }
    }
    Ok(())
}

#[test]
#[ignore = "fixed-domain grid-refinement research diagnostic; run in release mode"]
fn harris_subsonic_fixed_domain_grid_refinement() -> Result<(), String> {
    // Same finite domain and boundary conditions on every grid; this is not
    // a tunnel model or a physical validation. All lengths use body length.
    const AXIAL_EXTENSION: f64 = 0.5;
    const RADIAL_BOUNDARY: f64 = 1.25;
    const MACH: f64 = 0.9;
    const GAMMA: f64 = 1.4;
    const TOLERANCE: f64 = 1e-6;
    const MAX_ITERATIONS: usize = 40000;
    const STATIONS: [f64; 5] = [0.25, 0.375, 0.5, 0.625, 0.75];

    for intervals in [16_usize, 32, 64] {
        let dx = 1.0 / intervals as f64;
        let nx = 2 * intervals + 1;
        let nr = (RADIAL_BOUNDARY / dx + 0.5).round_ties_even() as usize;
        // The outer Dirichlet cell centre is fixed while dr approaches dx.
        let dr = RADIAL_BOUNDARY / (nr as f64 - 0.5);
        let source: Vec<_> = (0..nx)
            .map(|i| {
                let x = i as f64 * dx - AXIAL_EXTENSION;
                if (0.0..=1.0).contains(&x) {
                    harris_geometry(x).2
                } else {
                    0.0
                }
            })
            .collect();
        assert_eq!((source[0], source[1]), (0.0, 0.0));
        let mut grid = MixedGrid {
            phi: vec![vec![0.0; nr]; nx],
            axis_flux: source,
            dx,
            dr,
            mach: MACH,
            gamma: GAMMA,
        };
        let (iterations, residual) = grid
            .iterate(TOLERANCE, MAX_ITERATIONS)
            .map_err(|value| format!("intervals={intervals}: residual={value}"))?;
        println!(
            "intervals={intervals} dx={dx:.8} dr={dr:.8} iterations={iterations} residual={residual:.3e}"
        );
        for x in STATIONS {
            let i = ((x + AXIAL_EXTENSION) / dx).round() as usize;
            let (radius, slope, _) = harris_geometry(x);
            let cp = grid
                .surface_cp(i, radius, slope)
                .ok_or_else(|| format!("unavailable Cp at x/l={x}, intervals={intervals}"))?;
            println!("intervals={intervals} x/l={x:.3} cp={cp:.9}");
        }
    }
    Ok(())
}

fn interpolated_radial_derivative(values: &[f64], dr: f64, radius: f64) -> Option<f64> {
    if values.len() < 3 || !dr.is_finite() || dr <= 0.0 || !radius.is_finite() {
        return None;
    }
    let index = radius / dr - 1.0;
    if index < 0.0 || index > (values.len() - 2) as f64 {
        return None;
    }
    let j = (index.floor() as usize).min(values.len() - 3);
    let weight = index - j as f64;
    let left = (values[j + 1] - values[j]) / dr;
    let right = (values[j + 2] - values[j + 1]) / dr;
    let result = (1.0 - weight) * left + weight * right;
    result.is_finite().then_some(result)
}

#[test]
fn interpolated_radial_derivative_tracks_off_face_surface() -> Result<(), String> {
    const RADIUS: f64 = 0.05;
    const SLOPE: f64 = 0.2;
    const SOURCE: f64 = RADIUS * SLOPE;
    let mut previous_error = f64::INFINITY;
    for cells in [4_usize, 8, 16, 32] {
        let dr = RADIUS / (cells as f64 + 0.25);
        let values: Vec<_> = (0..cells + 3)
            .map(|k| SOURCE * (((k as f64 + 0.5) * dr) / RADIUS).ln())
            .collect();
        let derivative = interpolated_radial_derivative(&values, dr, RADIUS)
            .ok_or_else(|| format!("surface is not bracketed: cells={cells}"))?;
        let error = (derivative - SLOPE).abs();
        assert!(error < previous_error);
        println!("cells={cells} error={error:.9}");
        previous_error = error;
    }
    assert!(interpolated_radial_derivative(&[0.0, 1.0, 2.0], 1.0, 0.5).is_none());
    assert!(interpolated_radial_derivative(&[0.0, 1.0, 2.0], 0.0, 1.0).is_none());
    Ok(())
}

#[test]
#[ignore = "surface-condition research diagnostic; run in release mode"]
fn harris_subsonic_surface_impermeability_diagnostic() -> Result<(), String> {
    // Evaluate the linear slender-body condition phi_r = R' at the physical
    // surface by interpolating face derivatives. No radial extrapolation and
    // no claim that the singular axis-source approximation enforces it there.
    const EXTENSION: f64 = 0.5;
    const OUTER: f64 = 1.25;
    const MACH: f64 = 0.9;
    const GAMMA: f64 = 1.4;
    const TOLERANCE: f64 = 1e-6;
    const MAX_ITERATIONS: usize = 40000;
    const STATIONS: [f64; 5] = [0.125, 0.25, 0.625, 0.875, 0.9375];
    for intervals in [16_usize, 32, 64] {
        let dx = 1.0 / intervals as f64;
        let nx = 2 * intervals + 1;
        let nr = (OUTER / dx + 0.5).round_ties_even() as usize;
        let dr = OUTER / (nr as f64 - 0.5);
        let source = (0..nx)
            .map(|i| {
                let x = i as f64 * dx - EXTENSION;
                if (0.0..=1.0).contains(&x) {
                    harris_geometry(x).2
                } else {
                    0.0
                }
            })
            .collect();
        let mut grid = MixedGrid {
            phi: vec![vec![0.0; nr]; nx],
            axis_flux: source,
            dx,
            dr,
            mach: MACH,
            gamma: GAMMA,
        };
        let (_, residual) = grid
            .iterate(TOLERANCE, MAX_ITERATIONS)
            .map_err(|value| format!("intervals={intervals}: residual={value}"))?;
        for x in STATIONS {
            let i = ((x + EXTENSION) / dx).round() as usize;
            let (radius, slope, _) = harris_geometry(x);
            let Some(radial_velocity) = interpolated_radial_derivative(&grid.phi[i], dr, radius)
            else {
                println!(
                    "intervals={intervals} x/l={x:.4} unavailable: surface outside interpolable face range"
                );
                continue;
            };
            println!(
                "intervals={intervals} x/l={x:.4} radius={radius:.6} slope={slope:.6} phi_r={radial_velocity:.6} defect={:+.6} residual={residual:.3e}",
                radial_velocity - slope
            );
        }
    }
    Ok(())
}

#[test]
#[ignore = "boundary sensitivity research diagnostic; run in release mode"]
fn harris_subsonic_radial_boundary_sensitivity() -> Result<(), String> {
    // Hold the entire mesh spacing and axial domain fixed; only the location
    // of the outer zero-potential cell centre changes. Not a tunnel model.
    const BODY_INTERVALS: usize = 32;
    const AXIAL_EXTENSION: f64 = 0.5;
    const MACH: f64 = 0.9;
    const GAMMA: f64 = 1.4;
    const TOLERANCE: f64 = 1e-6;
    const MAX_ITERATIONS: usize = 40000;
    const STATIONS: [f64; 5] = [0.25, 0.375, 0.5, 0.625, 0.75];
    let dx = 1.0 / BODY_INTERVALS as f64;
    let dr = dx;
    let nx = 2 * BODY_INTERVALS + 1;
    for nr in [25_usize, 41, 65, 97] {
        let outer = (nr as f64 - 0.5) * dr;
        let source = (0..nx)
            .map(|i| {
                let x = i as f64 * dx - AXIAL_EXTENSION;
                if (0.0..=1.0).contains(&x) {
                    harris_geometry(x).2
                } else {
                    0.0
                }
            })
            .collect();
        let mut grid = MixedGrid {
            phi: vec![vec![0.0; nr]; nx],
            axis_flux: source,
            dx,
            dr,
            mach: MACH,
            gamma: GAMMA,
        };
        let (iterations, residual) = grid
            .iterate(TOLERANCE, MAX_ITERATIONS)
            .map_err(|value| format!("outer={outer}: residual={value}"))?;
        println!(
            "outer={outer:.6} dx={dx:.6} dr={dr:.6} iterations={iterations} residual={residual:.3e}"
        );
        for x in STATIONS {
            let i = ((x + AXIAL_EXTENSION) / dx).round() as usize;
            let (radius, slope, _) = harris_geometry(x);
            let cp = grid
                .surface_cp(i, radius, slope)
                .ok_or_else(|| format!("unavailable Cp at x/l={x}, outer={outer}"))?;
            println!("outer={outer:.6} x/l={x:.3} cp={cp:.9}");
        }
    }
    Ok(())
}

#[test]
#[ignore = "axial boundary sensitivity research diagnostic; run in release mode"]
fn harris_subsonic_axial_boundary_sensitivity() -> Result<(), String> {
    // Both axial boundaries move together while spacings and the radial
    // Dirichlet cell centre remain fixed. This does not represent tunnel walls.
    const BODY_INTERVALS: usize = 32;
    const RADIAL_CELLS: usize = 97;
    const MACH: f64 = 0.9;
    const GAMMA: f64 = 1.4;
    const TOLERANCE: f64 = 1e-6;
    const MAX_ITERATIONS: usize = 40000;
    const STATIONS: [f64; 5] = [0.25, 0.375, 0.5, 0.625, 0.75];
    let dx = 1.0 / BODY_INTERVALS as f64;
    let dr = dx;
    for extension_intervals in [16_usize, 32, 64, 96] {
        let extension = extension_intervals as f64 * dx;
        let nx = BODY_INTERVALS + 2 * extension_intervals + 1;
        let source = (0..nx)
            .map(|i| {
                let x = i as f64 * dx - extension;
                if (0.0..=1.0).contains(&x) {
                    harris_geometry(x).2
                } else {
                    0.0
                }
            })
            .collect();
        let mut grid = MixedGrid {
            phi: vec![vec![0.0; RADIAL_CELLS]; nx],
            axis_flux: source,
            dx,
            dr,
            mach: MACH,
            gamma: GAMMA,
        };
        let (iterations, residual) = grid
            .iterate(TOLERANCE, MAX_ITERATIONS)
            .map_err(|value| format!("extension={extension}: residual={value}"))?;
        println!(
            "extension={extension:.3} dx={dx:.6} dr={dr:.6} outer={:.6} iterations={iterations} residual={residual:.3e}",
            (RADIAL_CELLS as f64 - 0.5) * dr
        );
        for x in STATIONS {
            let i = ((x + extension) / dx).round() as usize;
            let (radius, slope, _) = harris_geometry(x);
            let cp = grid
                .surface_cp(i, radius, slope)
                .ok_or_else(|| format!("unavailable Cp at x/l={x}, extension={extension}"))?;
            println!("extension={extension:.3} x/l={x:.3} cp={cp:.9}");
        }
    }
    Ok(())
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

// Linear threshold positions for a decreasing profile, restricted to a
// fixed physical window. This diagnostic does not assert a physical shock.
fn transition_width(values: &[f64], dx: f64, start: f64, end: f64) -> Option<(f64, f64, f64)> {
    if start >= end || dx <= 0.0 || values.len() < 2 {
        return None;
    }
    let interpolate = |x: f64| {
        let index = x / dx - 0.5;
        if !index.is_finite() || index < 0.0 || index > (values.len() - 1) as f64 {
            return None;
        }
        let i = (index.floor() as usize).min(values.len() - 2);
        let t = index - i as f64;
        Some((1.0 - t) * values[i] + t * values[i + 1])
    };
    let (first, last) = (interpolate(start)?, interpolate(end)?);
    let drop = first - last;
    if !drop.is_finite() || drop <= 0.0 {
        return None;
    }
    let position = |fraction: f64| {
        let threshold = first - fraction * drop;
        (0..values.len() - 1).find_map(|i| {
            let (a, b) = (values[i], values[i + 1]);
            if a >= threshold && b < threshold {
                let x = (i as f64 + 0.5 + (a - threshold) / (a - b)) * dx;
                (start..=end).contains(&x).then_some(x)
            } else {
                None
            }
        })
    };
    let x10 = position(0.1)?;
    let x90 = position(0.9)?;
    (x90 >= x10).then_some((x10, x90, drop))
}

#[test]
fn transition_width_uses_face_coordinates_and_rejects_increasing_profiles() {
    let (x10, x90, drop) = transition_width(&[1.0, 1.0, 0.0, 0.0], 0.1, 0.05, 0.35).unwrap();
    assert!((x10 - 0.16).abs() < 1e-14);
    assert!((x90 - 0.24).abs() < 1e-14);
    assert_eq!(drop, 1.0);
    let (x10, x90, drop) = transition_width(&[1.0, 1.0, 0.0, 0.0], 0.1, 0.175, 0.225).unwrap();
    assert!((drop - 0.5).abs() < 1e-14);
    assert!((x10 - 0.18).abs() < 1e-14);
    assert!((x90 - 0.22).abs() < 1e-14);
    assert!(transition_width(&[0.0, 1.0], 0.1, 0.05, 0.15).is_none());
    assert!(transition_width(&[1.0, 0.0], 0.1, 0.0, 0.15).is_none());
}

#[test]
fn harris_mach_one_field_transition_diagnostic() {
    // Zero Dirichlet boundaries and the simplified sting do not match the
    // experiment. A descending sonic crossing is not proof of a shock.
    harris_mach_one_cases(&[
        (17, 0.75_f64),
        (33, 0.75),
        (49, 0.75),
        (65, 0.75),
        (17, 1.0),
        (33, 1.0),
        (49, 1.0),
        (65, 1.0),
        (17, 1.25),
        (33, 1.25),
        (49, 1.25),
        (65, 1.25),
    ]);
}

#[test]
#[ignore = "expensive fine-grid research diagnostic; run explicitly in release mode"]
fn harris_mach_one_fine_grid_diagnostic() {
    harris_mach_one_cases(&[(97, 0.75), (97, 1.0), (97, 1.25)]);
}

fn harris_mach_one_cases(cases: &[(usize, f64)]) {
    for &(n, outer) in cases {
        let dx = 1.0 / (n - 1) as f64;
        let nr = (outer / dx + 0.5).round_ties_even() as usize;
        let dr = outer / (nr as f64 - 0.5);
        let mut grid = MixedGrid {
            phi: vec![vec![0.0; nr]; n],
            axis_flux: (0..n).map(|i| harris_geometry(i as f64 * dx).2).collect(),
            dx,
            dr,
            mach: 1.0,
            gamma: 1.4,
        };
        const RESIDUAL_TOLERANCE: f64 = 1e-6;
        const MAX_ITERATIONS: usize = 40000;
        match grid.iterate(RESIDUAL_TOLERANCE, MAX_ITERATIONS) {
            Ok((iterations, residual)) => {
                for radius in [0.0625, 0.125, 0.25] {
                    let index = radius / dr - 0.5;
                    let k = index.floor() as usize;
                    let weight = index - k as f64;
                    let values: Vec<_> = grid
                        .phi
                        .iter()
                        .map(|row| (1.0 - weight) * row[k] + weight * row[k + 1])
                        .collect();
                    let velocity: Vec<_> = values.windows(2).map(|v| (v[1] - v[0]) / dx).collect();
                    let (x10, x90, drop) = transition_width(&velocity, dx, 0.6875, 0.875)
                        .expect("fixed-window transition must be measurable");
                    println!(
                        "  indicator n={n} outer={outer} r={radius} x10={x10:.6} x90={x90:.6} width={:.6} drop={drop:.6}",
                        x90 - x10
                    );
                    let crossings: Vec<_> = velocity
                        .windows(2)
                        .enumerate()
                        .filter(|(_, v)| v[0] > 0.0 && v[1] <= 0.0)
                        .map(|(i, _)| (i + 1) as f64 * dx)
                        .collect();
                    println!(
                        "n={n} outer={outer} r={radius} iterations={iterations} residual={residual:.3e} descending_sonic={crossings:?}"
                    );
                    assert!(!crossings.is_empty());
                    // Fixed physical windows bracket the observed transition;
                    // they are not extrapolated states or a shock-width estimate.
                    let samples: Vec<_> = [0.625, 0.6875, 0.75, 0.8125, 0.875]
                        .into_iter()
                        .map(|x| {
                            let i = ((x / dx) as usize).min(velocity.len() - 1);
                            (x, velocity[i])
                        })
                        .collect();
                    println!(
                        "  face velocity samples (x-node indexing, face at x+dx/2): {samples:?}"
                    );
                }
                let pressures: Vec<_> = [0.625, 0.6875, 0.75, 0.8125, 0.875]
                    .into_iter()
                    .map(|x| {
                        let (radius, slope, _) = harris_geometry(x);
                        (x, grid.surface_cp((x / dx).round() as usize, radius, slope))
                    })
                    .collect();
                println!("  surface Cp samples: {pressures:?}");
                for &(x, measured) in harris_data::TABLE_III_A_MACH_100_AFT {
                    let i = (x / dx).floor() as usize;
                    let node_cp = |j: usize| {
                        let (radius, slope, _) = harris_geometry(j as f64 * dx);
                        grid.surface_cp(j, radius, slope)
                    };
                    if let (Some(left), Some(right)) = (node_cp(i), node_cp(i + 1)) {
                        let t = x / dx - i as f64;
                        let calculated = (1.0 - t) * left + t * right;
                        println!(
                            "  Mach1 n={n} outer={outer} x={x:.3} measured={measured:.3} calculated={calculated:.6} delta={:+.6}",
                            calculated - measured
                        );
                    } else {
                        println!(
                            "  Mach1 n={n} outer={outer} x={x:.3} unavailable without extrapolation"
                        );
                    }
                }
            }
            Err(residual) => panic!("n={n} did not converge: residual={residual:.3e}"),
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
    let mut comparisons = Vec::new();
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
        let mut count = 0;
        let mut sum_squared = 0.0;
        let mut max_error = 0.0_f64;
        let mut errors = vec![None; harris_data::TABLE_III_A_MACH_090.len()];
        for (station, &(x, measured)) in harris_data::TABLE_III_A_MACH_090.iter().enumerate() {
            let i = (x / dx).floor() as usize;
            if i == 0 || i + 2 >= n {
                continue;
            }
            let node_cp = |j: usize| {
                let (radius, slope, _) = harris_geometry(j as f64 * dx);
                grid.surface_cp(j, radius, slope)
            };
            let (Some(left), Some(right)) = (node_cp(i), node_cp(i + 1)) else {
                continue;
            };
            let t = x / dx - i as f64;
            let calculated = (1.0 - t) * left + t * right;
            let difference = calculated - measured;
            errors[station] = Some(difference);
            println!(
                "n={n} outer={outer} x/l={x:.3} Cp_calc={calculated:.6} Cp_measured={measured:.3} difference={difference:+.6}"
            );
            assert!(calculated.is_finite());
            count += 1;
            sum_squared += difference * difference;
            max_error = max_error.max(difference.abs());
        }
        assert!(count > 0);
        println!(
            "n={n} outer={outer} compared={count}/{} rmse={:.6} max_abs={max_error:.6}",
            harris_data::TABLE_III_A_MACH_090.len(),
            (sum_squared / count as f64).sqrt()
        );
        comparisons.push((n, outer, errors));
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
    let common: Vec<_> = (0..harris_data::TABLE_III_A_MACH_090.len())
        .filter(|&i| comparisons.iter().all(|(_, _, errors)| errors[i].is_some()))
        .collect();
    assert!(!common.is_empty());
    println!(
        "common stations: {:?}",
        common
            .iter()
            .map(|&i| harris_data::TABLE_III_A_MACH_090[i].0)
            .collect::<Vec<_>>()
    );
    for (n, outer, errors) in &comparisons {
        let squared: f64 = common.iter().map(|&i| errors[i].unwrap().powi(2)).sum();
        let max = common
            .iter()
            .map(|&i| errors[i].unwrap().abs())
            .fold(0.0_f64, f64::max);
        println!(
            "common n={n} outer={outer} count={} rmse={:.6} max_abs={max:.6}",
            common.len(),
            (squared / common.len() as f64).sqrt()
        );
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
