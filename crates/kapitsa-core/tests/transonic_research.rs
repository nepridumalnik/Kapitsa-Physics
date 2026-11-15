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

fn backward_derivative(previous_two: f64, previous: f64, current: f64, dx: f64) -> f64 {
    (3.0 * current - 4.0 * previous + previous_two) / (2.0 * dx)
}

#[test]
fn backward_derivative_is_second_order_on_smooth_field() {
    const X: f64 = 0.9;
    let mut previous_error = f64::INFINITY;
    for dx in [0.1_f64, 0.05, 0.025] {
        let derivative = backward_derivative((X - 2.0 * dx).exp(), (X - dx).exp(), X.exp(), dx);
        let error = (derivative - X.exp()).abs();
        assert!(error < previous_error / 3.5);
        previous_error = error;
    }
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
fn cutoff_cell_source_average_preserves_cross_section_change() {
    // Integral of R R' over a cell is exactly half the change in R²,
    // including a cell crossed by the continuous-radius slope discontinuity.
    const CUTOFF: f64 = 36.0 / 38.240;
    const ROUNDING_TOLERANCE: f64 = 1e-15;
    for intervals in [32_usize, 64, 96, 128] {
        let dx = 1.0 / intervals as f64;
        let i = (CUTOFF / dx).round() as usize;
        let x = i as f64 * dx;
        let left = harris_geometry(x - dx / 2.0).0;
        let right = harris_geometry(x + dx / 2.0).0;
        let average = (right * right - left * left) / (2.0 * dx);
        let point = harris_geometry(x).2;
        let integrated = average * dx;
        assert!((integrated - (right * right - left * left) / 2.0).abs() < ROUNDING_TOLERANCE);
        assert!(x - dx / 2.0 < CUTOFF && x + dx / 2.0 > CUTOFF);
        println!(
            "intervals={intervals} offset_in_cells={:.6} point={point:.9} average={average:.9} difference={:+.9}",
            (CUTOFF - x) / dx,
            average - point
        );
    }
}

fn harris_conservative_source(x: f64, dx: f64) -> f64 {
    let left = harris_geometry(x - 0.5 * dx).0;
    let right = harris_geometry(x + 0.5 * dx).0;
    (right * right - left * left) / (2.0 * dx)
}

#[test]
fn conservative_source_telescopes_across_nose_and_cutoff() {
    const CUTOFF: f64 = 36.0 / 38.240;
    for intervals in [32_usize, 64, 96, 128] {
        let dx = 1.0 / intervals as f64;
        let source: Vec<_> = (0..=intervals)
            .map(|i| harris_conservative_source(i as f64 * dx, dx))
            .collect();
        let integrated = source.iter().sum::<f64>() * dx;
        let expected = (harris_geometry(1.0 + 0.5 * dx).0.powi(2)
            - harris_geometry(-0.5 * dx).0.powi(2))
            / 2.0;
        let cutoff_cell = (CUTOFF / dx).round() as usize;
        let cutoff_left = cutoff_cell as f64 * dx - 0.5 * dx;
        let cutoff_right = cutoff_left + dx;
        let cutoff_bracketed = cutoff_left < CUTOFF && cutoff_right > CUTOFF;
        assert!(cutoff_bracketed);
        assert!((integrated - expected).abs() < 2e-15);
        println!(
            "intervals={intervals} integrated={integrated:.15e} expected={expected:.15e} cutoff_cell={cutoff_cell} cutoff_bracket=[{cutoff_left:.9},{cutoff_right:.9}] bracketed={cutoff_bracketed}"
        );
    }
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

#[test]
#[ignore = "low-Mach profile and balance diagnostic; run in release mode"]
fn harris_low_mach_conservative_profile_and_balance() -> Result<(), String> {
    // Finite idealized domain; Harris Mach 0.9 stations provide coordinates
    // only, not reference pressures for this Mach 0.3 calculation.
    const EXTENSION: f64 = 0.5;
    const OUTER: f64 = 1.25;
    const MACH: f64 = 0.3;
    const GAMMA: f64 = 1.4;
    const CUTOFF: f64 = 36.0 / 38.240;
    let mut profiles = Vec::new();
    for intervals in [16_usize, 32, 64, 96] {
        let dx = 1.0 / intervals as f64;
        let nx = 2 * intervals + 1;
        let nr = (OUTER / dx + 0.5).round() as usize;
        let dr = OUTER / (nr as f64 - 0.5);
        let mut grid = MixedGrid {
            phi: vec![vec![0.0; nr]; nx],
            axis_flux: (0..nx)
                .map(|i| harris_conservative_source(i as f64 * dx - EXTENSION, dx))
                .collect(),
            dx,
            dr,
            mach: MACH,
            gamma: GAMMA,
        };
        let (iterations, residual) = grid
            .iterate(1e-8, 100_000)
            .map_err(|value| format!("intervals={intervals} residual={value}"))?;
        let mut volume = 0.0;
        let mut boundary = 0.0;
        for i in 2..nx - 1 {
            for k in 0..nr - 1 {
                volume += grid.residual(i, k) * (k as f64 + 0.5) * dr * dx * dr;
            }
            boundary += dx
                * ((nr - 1) as f64 * (grid.phi[i][nr - 1] - grid.phi[i][nr - 2])
                    - grid.axis_flux[i]);
        }
        for k in 0..nr - 1 {
            let slope = |j: usize| (grid.phi[j][k] - grid.phi[j - 1][k]) / dx;
            boundary += (k as f64 + 0.5)
                * dr
                * dr
                * (sonic_flux(slope(nx - 2), slope(nx - 1), MACH, GAMMA)
                    - sonic_flux(slope(1), slope(2), MACH, GAMMA));
        }
        assert!((volume - boundary).abs() < 1e-12 * boundary.abs().max(1e-6));
        println!(
            "balance intervals={intervals} dx={dx:.9} dr={dr:.9} residual={residual:.3e} iterations={iterations} volume={volume:.9e} boundary={boundary:.9e} mismatch={:.3e}",
            (volume - boundary).abs()
        );
        let mut profile = Vec::new();
        for &(x, _) in harris_data::TABLE_III_A_MACH_090 {
            let index = (x + EXTENSION) / dx;
            let i = index.floor() as usize;
            let t = index - i as f64;
            let node = |j: usize| {
                let node_x = j as f64 * dx - EXTENSION;
                let (radius, slope, _) = harris_geometry(node_x);
                grid.surface_cp(j, radius, slope)
            };
            // Reject interpolation across the physical corner; central Cp
            // stencils can still span it and are explicitly reported.
            let left_x = i as f64 * dx - EXTENSION;
            let cp = if left_x < CUTOFF && left_x + dx >= CUTOFF {
                None
            } else {
                node(i).zip(node(i + 1)).map(|(a, b)| (1.0 - t) * a + t * b)
            };
            println!(
                "profile intervals={intervals} x={x:.3} cp={cp:?} stencil_crosses_cutoff={}",
                left_x - dx < CUTOFF && left_x + 2.0 * dx >= CUTOFF
            );
            if x >= 0.89 && left_x + dx < CUTOFF {
                let backward_cp = |j: usize| -> Option<f64> {
                    let body_x = j as f64 * dx - EXTENSION;
                    if j < 2 || body_x >= CUTOFF {
                        return None;
                    }
                    let (radius, slope, _) = harris_geometry(body_x);
                    let index = radius / dr - 0.5;
                    if !(0.0..=(nr - 1) as f64).contains(&index) {
                        return None;
                    }
                    let k = (index.floor() as usize).min(nr - 2);
                    let weight = index - k as f64;
                    let derivative = |r: usize| {
                        backward_derivative(
                            grid.phi[j - 2][r],
                            grid.phi[j - 1][r],
                            grid.phi[j][r],
                            dx,
                        )
                    };
                    let value = -2.0
                        * ((1.0 - weight) * derivative(k) + weight * derivative(k + 1))
                        - slope * slope;
                    value.is_finite().then_some(value)
                };
                let backward = backward_cp(i)
                    .zip(backward_cp(i + 1))
                    .map(|(a, b)| (1.0 - t) * a + t * b);
                println!(
                    "reconstruction intervals={intervals} x={x:.3} central={cp:?} backward={backward:?}"
                );
            }
            profile.push(cp);
        }
        profiles.push(profile);
    }
    let common: Vec<_> = (0..harris_data::TABLE_III_A_MACH_090.len())
        .filter(|&i| profiles.iter().all(|p| p[i].is_some()))
        .collect();
    if common.is_empty() {
        return Err("no common stations".to_string());
    }
    for pair in profiles.windows(2) {
        let mut max_delta = 0.0_f64;
        for &i in &common {
            let (Some(a), Some(b)) = (pair[0][i], pair[1][i]) else {
                return Err("missing common station".to_string());
            };
            max_delta = max_delta.max((b - a).abs());
        }
        println!("common count={} max_delta={max_delta:.9}", common.len());
    }
    let fine_common: Vec<_> = (0..harris_data::TABLE_III_A_MACH_090.len())
        .filter(|&i| profiles[1..].iter().all(|p| p[i].is_some()))
        .collect();
    for pair in profiles[1..].windows(2) {
        let mut max_delta = 0.0_f64;
        let mut max_x = 0.0;
        for &i in &fine_common {
            let (Some(a), Some(b)) = (pair[0][i], pair[1][i]) else {
                return Err("missing fine common station".to_string());
            };
            if (b - a).abs() > max_delta {
                max_delta = (b - a).abs();
                max_x = harris_data::TABLE_III_A_MACH_090[i].0;
            }
        }
        println!(
            "fine_common count={} max_delta={max_delta:.9} max_x={max_x:.3}",
            fine_common.len()
        );
    }
    Ok(())
}

#[test]
#[ignore = "full pressure-profile research diagnostic; run in release mode"]
fn harris_subsonic_extended_domain_pressure_profile() -> Result<(), String> {
    // Fixed finite boundaries: not the measured tunnel, sting or viscous flow.
    // Experimental uncertainty is not established. No acceptance threshold.
    const EXTENSION: f64 = 2.0;
    const OUTER: f64 = 3.015625;
    const MACH: f64 = 0.9;
    const GAMMA: f64 = 1.4;
    const TOLERANCE: f64 = 1e-6;
    const MAX_ITERATIONS: usize = 60000;
    const CUTOFF: f64 = 36.0 / 38.240;
    let mut profiles = Vec::new();
    for intervals in [16_usize, 32, 64] {
        let dx = 1.0 / intervals as f64;
        let nx = 5 * intervals + 1;
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
        let (iterations, residual) = grid
            .iterate(TOLERANCE, MAX_ITERATIONS)
            .map_err(|value| format!("intervals={intervals}: residual={value}"))?;
        println!(
            "intervals={intervals} dx={dx:.8} dr={dr:.8} iterations={iterations} residual={residual:.3e}"
        );
        let mut count = 0_usize;
        let mut sum_squared = 0.0;
        let mut profile = vec![None; harris_data::TABLE_III_A_MACH_090.len()];
        for (station, &(x, measured)) in harris_data::TABLE_III_A_MACH_090.iter().enumerate() {
            let index = (x + EXTENSION) / dx;
            let i = index.floor() as usize;
            let node_cp = |j: usize| {
                let body_x = j as f64 * dx - EXTENSION;
                let (radius, slope, _) = harris_geometry(body_x);
                grid.surface_cp(j, radius, slope)
            };
            let (Some(left), Some(right)) = (node_cp(i), node_cp(i + 1)) else {
                println!(
                    "intervals={intervals} x/l={x:.3} unavailable without radial extrapolation"
                );
                continue;
            };
            let t = index - i as f64;
            let cp = (1.0 - t) * left + t * right;
            profile[station] = Some(cp);
            let left_x = i as f64 * dx - EXTENSION;
            let right_x = left_x + dx;
            if left_x < CUTOFF && right_x >= CUTOFF {
                println!(
                    "cutoff crossing: intervals={intervals} x/l={x:.3} bracket=[{left_x:.6},{right_x:.6}] left_cp={left:.6} right_cp={right:.6} interpolated={cp:.6}"
                );
            }
            if left_x - dx < CUTOFF && right_x + dx >= CUTOFF {
                println!(
                    "cutoff derivative stencil: intervals={intervals} x/l={x:.3} node_bracket=[{left_x:.6},{right_x:.6}] derivative_span=[{:.6},{:.6}] cp={cp:.6}",
                    left_x - dx,
                    right_x + dx
                );
            }
            if intervals >= 32 && x >= 0.9 {
                let backward_cp = |j: usize| -> Option<f64> {
                    let body_x = j as f64 * dx - EXTENSION;
                    if j < 2 || body_x >= CUTOFF {
                        return None;
                    }
                    let (radius, slope, _) = harris_geometry(body_x);
                    let radial_index = radius / dr - 0.5;
                    if radial_index < 0.0 || radial_index > (nr - 1) as f64 {
                        return None;
                    }
                    let k = (radial_index.floor() as usize).min(nr - 2);
                    let weight = radial_index - k as f64;
                    let derivative = |radial: usize| {
                        backward_derivative(
                            grid.phi[j - 2][radial],
                            grid.phi[j - 1][radial],
                            grid.phi[j][radial],
                            dx,
                        )
                    };
                    let cp = -2.0 * ((1.0 - weight) * derivative(k) + weight * derivative(k + 1))
                        - slope.powi(2);
                    cp.is_finite().then_some(cp)
                };
                if let (Some(a), Some(b)) = (backward_cp(i), backward_cp(i + 1)) {
                    let one_sided = (1.0 - t) * a + t * b;
                    println!(
                        "one-sided intervals={intervals} x/l={x:.3} central={cp:.6} backward={one_sided:.6} difference={:+.6}",
                        one_sided - cp
                    );
                }
            }
            let difference = cp - measured;
            sum_squared += difference * difference;
            count += 1;
            println!(
                "intervals={intervals} x/l={x:.3} cp={cp:.6} measured={measured:.3} delta={difference:+.6}"
            );
        }
        if count == 0 {
            return Err(format!("no available stations: intervals={intervals}"));
        }
        println!(
            "intervals={intervals} available={count}/{} rmse={:.6}",
            harris_data::TABLE_III_A_MACH_090.len(),
            (sum_squared / count as f64).sqrt()
        );
        profiles.push((intervals, profile));
    }
    let common: Vec<_> = (0..harris_data::TABLE_III_A_MACH_090.len())
        .filter(|&i| profiles.iter().all(|(_, profile)| profile[i].is_some()))
        .collect();
    if common.is_empty() {
        return Err("no common stations".to_string());
    }
    println!("common station count={}", common.len());
    for (intervals, profile) in &profiles {
        let mut squared = 0.0;
        for &i in &common {
            let cp = profile[i].ok_or_else(|| format!("missing common station {i}"))?;
            squared += (cp - harris_data::TABLE_III_A_MACH_090[i].1).powi(2);
        }
        println!(
            "common intervals={intervals} rmse={:.9}",
            (squared / common.len() as f64).sqrt()
        );
    }
    for pair in profiles.windows(2) {
        let mut max_difference = 0.0_f64;
        for &i in &common {
            let left = pair[0].1[i].ok_or_else(|| format!("missing station {i}"))?;
            let right = pair[1].1[i].ok_or_else(|| format!("missing station {i}"))?;
            max_difference = max_difference.max((right - left).abs());
        }
        println!(
            "common grid difference {} -> {} max_abs={max_difference:.9}",
            pair[0].0, pair[1].0
        );
    }
    Ok(())
}

#[test]
#[ignore = "synthetic sting sensitivity diagnostic; run in release mode"]
fn harris_subsonic_synthetic_sting_sensitivity() -> Result<(), String> {
    // This prescribed taper is a sensitivity case, not measured sting geometry.
    // Both continuations share the Harris body before its truncation.
    const CUTOFF: f64 = 36.0 / 38.240;
    const TAPER_LENGTH: f64 = 0.5;
    const EXTENSION: f64 = 1.0;
    const OUTER: f64 = 1.25;
    const MACH: f64 = 0.9;
    const GAMMA: f64 = 1.4;
    const TOLERANCE: f64 = 1e-6;
    const MAX_ITERATIONS: usize = 40000;
    const STATIONS: [f64; 4] = [0.25, 0.5, 0.75, 0.875];
    let cutoff_radius = harris_geometry(CUTOFF).0;
    for intervals in [16_usize, 32, 64] {
        let dx = 1.0 / intervals as f64;
        let nx = 3 * intervals + 1;
        let nr = (OUTER / dx + 0.5).round_ties_even() as usize;
        let dr = OUTER / (nr as f64 - 0.5);
        for taper in [false, true] {
            let source = (0..nx)
                .map(|i| {
                    let x = i as f64 * dx - EXTENSION;
                    if (0.0..CUTOFF).contains(&x) {
                        harris_geometry(x).2
                    } else if taper && (CUTOFF..CUTOFF + TAPER_LENGTH).contains(&x) {
                        let slope = -cutoff_radius / TAPER_LENGTH;
                        let radius = cutoff_radius + slope * (x - CUTOFF);
                        radius * slope
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
            let (iterations, residual) =
                grid.iterate(TOLERANCE, MAX_ITERATIONS).map_err(|value| {
                    format!("intervals={intervals} taper={taper}: residual={value}")
                })?;
            println!(
                "intervals={intervals} taper={taper} iterations={iterations} residual={residual:.3e}"
            );
            for x in STATIONS {
                let i = ((x + EXTENSION) / dx).round() as usize;
                let (radius, slope, _) = harris_geometry(x);
                let cp = grid
                    .surface_cp(i, radius, slope)
                    .ok_or_else(|| format!("unavailable Cp: x={x} intervals={intervals}"))?;
                println!("intervals={intervals} taper={taper} x={x:.3} cp={cp:.9}");
            }
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
    const MAX_ITERATIONS: usize = 60000;
    const STATIONS: [f64; 5] = [0.125, 0.25, 0.625, 0.875, 0.9375];
    for intervals in [16_usize, 32, 64, 96] {
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
#[ignore = "equation-type diagnostic; run in release mode"]
fn harris_subsonic_equation_type_diagnostic() -> Result<(), String> {
    // Classify the backward axial face slopes used by the numerical flux,
    // including points inside the idealized body: this is not a flow map.
    const EXTENSION: f64 = 0.5;
    const OUTER: f64 = 1.25;
    const GAMMA: f64 = 1.4;
    const TOLERANCE: f64 = 1e-6;
    const MAX_ITERATIONS: usize = 60_000;
    for mach in [0.3_f64, 0.9] {
        for intervals in [32_usize, 64, 96] {
            let dx = 1.0 / intervals as f64;
            let nx = 2 * intervals + 1;
            let nr = (OUTER / dx + 0.5).round() as usize;
            let dr = OUTER / (nr as f64 - 0.5);
            let mut grid = MixedGrid {
                phi: vec![vec![0.0; nr]; nx],
                axis_flux: (0..nx)
                    .map(|i| harris_conservative_source(i as f64 * dx - EXTENSION, dx))
                    .collect(),
                dx,
                dr,
                mach,
                gamma: GAMMA,
            };
            let (iterations, residual) = grid
                .iterate(TOLERANCE, MAX_ITERATIONS)
                .map_err(|value| format!("mach={mach} intervals={intervals} residual={value}"))?;
            let mut minimum = f64::INFINITY;
            let mut min_location = (0.0, 0.0);
            let mut negative = 0_usize;
            let mut count = 0_usize;
            let mut transitions = 0_usize;
            for k in 0..nr - 1 {
                let r = (k as f64 + 0.5) * dr;
                let mut previous: Option<(f64, bool)> = None;
                for i in 2..nx - 1 {
                    let x = (i as f64 - 0.5) * dx - EXTENSION;
                    let u = (grid.phi[i][k] - grid.phi[i - 1][k]) / dx;
                    let lambda = 1.0 - mach * mach - mach * mach * (GAMMA + 1.0) * u;
                    if !lambda.is_finite() {
                        return Err(format!("non-finite lambda at mach={mach} x={x} r={r}"));
                    }
                    if lambda < minimum {
                        minimum = lambda;
                        min_location = (x, r);
                    }
                    let hyperbolic = lambda < 0.0;
                    negative += usize::from(hyperbolic);
                    count += 1;
                    if let Some((previous_x, previous_hyperbolic)) = previous {
                        if previous_hyperbolic != hyperbolic {
                            transitions += 1;
                            println!(
                                "transition mach={mach} intervals={intervals} r={r:.9} x_bracket=[{previous_x:.9},{x:.9}] from_hyperbolic={previous_hyperbolic}"
                            );
                        }
                    }
                    previous = Some((x, hyperbolic));
                }
            }
            println!(
                "type mach={mach} intervals={intervals} dx={dx:.9} dr={dr:.9} min_lambda={minimum:.9} min_x={:.9} min_r={:.9} negative={negative}/{count} fraction={:.9} transitions={transitions} iterations={iterations} residual={residual:.3e}",
                min_location.0,
                min_location.1,
                negative as f64 / count as f64
            );
        }
    }
    Ok(())
}

#[test]
#[ignore = "source discretization sensitivity diagnostic; run in release mode"]
fn harris_subsonic_cutoff_source_discretization() -> Result<(), String> {
    // Fixed finite domain and identical initial conditions; averaging changes
    // source discretization only, not the geometry or physical closure.
    const EXTENSION: f64 = 0.5;
    const OUTER: f64 = 1.25;
    const CUTOFF: f64 = 36.0 / 38.240;
    const MACH: f64 = 0.9;
    const GAMMA: f64 = 1.4;
    const TOLERANCE: f64 = 1e-6;
    const MAX_ITERATIONS: usize = 60000;
    const STATIONS: [f64; 2] = [0.875, 0.9375];
    for intervals in [32_usize, 64, 96] {
        let dx = 1.0 / intervals as f64;
        let nx = 2 * intervals + 1;
        let nr = (OUTER / dx + 0.5).round_ties_even() as usize;
        let dr = OUTER / (nr as f64 - 0.5);
        for mode in ["point", "cutoff_cell", "all_cells"] {
            let mut changed_cells = 0;
            let source = (0..nx)
                .map(|i| {
                    let x = i as f64 * dx - EXTENSION;
                    let crosses_cutoff = x - 0.5 * dx < CUTOFF && x + 0.5 * dx > CUTOFF;
                    if mode == "all_cells" || (mode == "cutoff_cell" && crosses_cutoff) {
                        if mode == "cutoff_cell" {
                            changed_cells += 1;
                        }
                        harris_conservative_source(x, dx)
                    } else if (0.0..=1.0).contains(&x) {
                        harris_geometry(x).2
                    } else {
                        0.0
                    }
                })
                .collect();
            if mode == "cutoff_cell" {
                assert_eq!(changed_cells, 1);
            }
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
                .map_err(|value| format!("intervals={intervals} mode={mode}: residual={value}"))?;
            println!(
                "intervals={intervals} mode={mode} cutoff_phase={:.6} iterations={iterations} residual={residual:.3e}",
                CUTOFF / dx - (CUTOFF / dx).round()
            );
            for x in STATIONS {
                let i = ((x + EXTENSION) / dx).round() as usize;
                let (radius, slope, _) = harris_geometry(x);
                let cp = grid.surface_cp(i, radius, slope);
                let defect = interpolated_radial_derivative(&grid.phi[i], dr, radius)
                    .map(|radial_velocity| radial_velocity - slope);
                println!(
                    "intervals={intervals} mode={mode} x/l={x:.4} cp={cp:?} linear_surface_defect={defect:?}"
                );
            }
        }
    }
    Ok(())
}

#[test]
#[ignore = "near-cutoff boundary sensitivity diagnostic; run in release mode"]
fn harris_subsonic_upstream_cutoff_boundaries() -> Result<(), String> {
    // Shift boundaries by whole cells, retaining the source grid phase and
    // both spacings. These finite Dirichlet boundaries are not tunnel walls.
    const STATION: f64 = 0.9;
    const CUTOFF: f64 = 36.0 / 38.240;
    const EXTENSION: f64 = 0.5;
    const OUTER: f64 = 1.25;
    const TOLERANCE: f64 = 1e-6;
    const MAX_ITERATIONS: usize = 100_000;
    for axial_intervals in [128_usize, 129] {
        let dx = 2.0 / axial_intervals as f64;
        let base_nr = (OUTER / dx + 0.5).round() as usize;
        let dr = OUTER / (base_nr as f64 - 0.5);
        let extra_x = (0.5 / dx).round() as usize;
        let extra_r = (0.75 / dr).round() as usize;
        let farther_x = (1.5 / dx).round() as usize;
        let farther_r = (1.75 / dr).round() as usize;
        for (label, added_x, added_r) in [
            ("baseline", 0, 0),
            ("axial", extra_x, 0),
            ("axial_far", farther_x, 0),
            ("radial", 0, extra_r),
            ("radial_far", 0, farther_r),
        ] {
            let nx = axial_intervals + 1 + 2 * added_x;
            let nr = base_nr + added_r;
            let origin = -EXTENSION - added_x as f64 * dx;
            let source = (0..nx)
                .map(|i| {
                    // Evaluate with the original indexing to keep node positions
                    // identical even when the left boundary moves.
                    let x = (i as f64 - added_x as f64) * dx - EXTENSION;
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
                mach: 0.9,
                gamma: 1.4,
            };
            let (iterations, residual) =
                grid.iterate(TOLERANCE, MAX_ITERATIONS).map_err(|value| {
                    format!("grid={axial_intervals} boundary={label}: residual={value}")
                })?;
            let index = (STATION + EXTENSION) / dx;
            let base_i = index.floor() as usize;
            let i = base_i + added_x;
            let weight = index - base_i as f64;
            assert!((base_i + 2) as f64 * dx - EXTENSION < CUTOFF);
            let (radius, slope, _) = harris_geometry(STATION);
            let cp = grid
                .surface_cp(i, radius, slope)
                .zip(grid.surface_cp(i + 1, radius, slope))
                .map(|(a, b)| (1.0 - weight) * a + weight * b)
                .ok_or_else(|| {
                    format!("Cp unavailable: grid={axial_intervals} boundary={label}")
                })?;
            let defect = interpolated_radial_derivative(&grid.phi[i], dr, radius)
                .zip(interpolated_radial_derivative(&grid.phi[i + 1], dr, radius))
                .map(|(a, b)| (1.0 - weight) * a + weight * b - slope)
                .ok_or_else(|| {
                    format!(
                        "surface derivative unavailable: grid={axial_intervals} boundary={label}"
                    )
                })?;
            println!(
                "grid={axial_intervals} boundary={label} x_min={origin:.6} x_max={:.6} outer={:.6} dx={dx:.9} dr={dr:.9} cp={cp:.9} defect={defect:+.9} iterations={iterations} residual={residual:.3e}",
                origin + (nx - 1) as f64 * dx,
                (nr as f64 - 0.5) * dr
            );
        }
    }
    Ok(())
}

fn quadratic_axial_interpolation(left: f64, center: f64, right: f64, t: f64) -> f64 {
    0.5 * t * (t - 1.0) * left + (1.0 - t * t) * center + 0.5 * t * (t + 1.0) * right
}

#[test]
fn quadratic_axial_interpolation_reproduces_quadratic() {
    for t in [0.0_f64, 0.2, 0.75, 1.0] {
        let value = quadratic_axial_interpolation(6.0, 3.0, 4.0, t);
        assert!((value - (2.0 * t * t - t + 3.0)).abs() < 1e-14);
    }
}

#[test]
#[ignore = "shifted-grid phase diagnostic; run in release mode"]
fn harris_subsonic_shifted_grid_phase() -> Result<(), String> {
    // A uniform shift retains dx and the physical body, but necessarily moves
    // both axial Dirichlet nodes and the nose phase; this is not a pure cutoff
    // perturbation or a comparison at exactly identical axial boundaries.
    const EXTENSION: f64 = 0.5;
    const OUTER: f64 = 1.25;
    const STATION: f64 = 0.9;
    const CUTOFF: f64 = 36.0 / 38.240;
    const TOLERANCE: f64 = 1e-6;
    const MAX_ITERATIONS: usize = 100_000;
    for intervals in [64_usize, 96] {
        let dx = 1.0 / intervals as f64;
        let nx = 2 * intervals + 1;
        let nr = (OUTER / dx + 0.5).round() as usize;
        let dr = OUTER / (nr as f64 - 0.5);
        for (shift, mode) in [
            (0.0_f64, "point"),
            (0.0, "nose"),
            (0.0, "all"),
            (0.5, "point"),
            (0.5, "nose"),
            (0.5, "all"),
        ] {
            let origin = -EXTENSION + shift * dx;
            let nose_cell = ((0.25 * dx - origin) / dx + 0.5).floor() as usize;
            let mut modified = 0;
            let source = (0..nx)
                .map(|i| {
                    let x = origin + i as f64 * dx;
                    // Average the first cell with positive axial overlap with
                    // [0, dx/2], including a nose exactly on a cell face.
                    if mode == "all" {
                        harris_conservative_source(x, dx)
                    } else if mode == "nose" && i == nose_cell {
                        modified += 1;
                        let radius_at = |position: f64| {
                            if position <= 0.0 {
                                0.0
                            } else {
                                harris_geometry(position).0
                            }
                        };
                        let left = radius_at(x - 0.5 * dx);
                        let right = radius_at(x + 0.5 * dx);
                        (right * right - left * left) / (2.0 * dx)
                    } else if (0.0..=1.0).contains(&x) {
                        harris_geometry(x).2
                    } else {
                        0.0
                    }
                })
                .collect();
            if mode == "nose" {
                assert_eq!(modified, 1);
            }
            let mut grid = MixedGrid {
                phi: vec![vec![0.0; nr]; nx],
                axis_flux: source,
                dx,
                dr,
                mach: 0.9,
                gamma: 1.4,
            };
            let (iterations, residual) =
                grid.iterate(TOLERANCE, MAX_ITERATIONS).map_err(|value| {
                    format!("intervals={intervals} shift={shift}: residual={value}")
                })?;
            let index = (STATION - origin) / dx;
            let i = index.floor() as usize;
            let weight = index - i as f64;
            assert!(origin + (i + 2) as f64 * dx < CUTOFF);
            let (radius, slope, _) = harris_geometry(STATION);
            let cp = grid
                .surface_cp(i, radius, slope)
                .zip(grid.surface_cp(i + 1, radius, slope))
                .map(|(a, b)| (1.0 - weight) * a + weight * b)
                .ok_or_else(|| format!("Cp unavailable: intervals={intervals} shift={shift}"))?;
            let defect = interpolated_radial_derivative(&grid.phi[i], dr, radius)
                .zip(interpolated_radial_derivative(&grid.phi[i + 1], dr, radius))
                .map(|(a, b)| (1.0 - weight) * a + weight * b - slope)
                .ok_or_else(|| {
                    format!("surface derivative unavailable: intervals={intervals} shift={shift}")
                })?;
            let cp_left = grid.surface_cp(i - 1, radius, slope).ok_or_else(|| {
                format!("quadratic Cp unavailable: intervals={intervals} shift={shift}")
            })?;
            let cp_center = grid.surface_cp(i, radius, slope).ok_or_else(|| {
                format!("quadratic Cp unavailable: intervals={intervals} shift={shift}")
            })?;
            let cp_right = grid.surface_cp(i + 1, radius, slope).ok_or_else(|| {
                format!("quadratic Cp unavailable: intervals={intervals} shift={shift}")
            })?;
            let quadratic_cp = quadratic_axial_interpolation(cp_left, cp_center, cp_right, weight);
            let radial = |j: usize| interpolated_radial_derivative(&grid.phi[j], dr, radius);
            let quadratic_defect = match (radial(i - 1), radial(i), radial(i + 1)) {
                (Some(left), Some(center), Some(right)) => {
                    quadratic_axial_interpolation(left, center, right, weight) - slope
                }
                _ => {
                    return Err(format!(
                        "quadratic radial derivative unavailable: intervals={intervals} shift={shift}"
                    ));
                }
            };
            let cutoff_index = (CUTOFF - origin) / dx;
            println!(
                "intervals={intervals} shift={shift:.1} mode={mode} dx={dx:.9} dr={dr:.9} x_min={origin:.9} x_max={:.9} cutoff_phase={:+.6} bracket=[{:.9},{:.9}] cp={cp:.9} defect={defect:+.9} iterations={iterations} residual={residual:.3e}",
                origin + (nx - 1) as f64 * dx,
                cutoff_index - cutoff_index.round(),
                origin + i as f64 * dx,
                origin + (i + 1) as f64 * dx
            );
            println!(
                "interpolation intervals={intervals} shift={shift:.1} mode={mode} quadratic_cp={quadratic_cp:.9} cp_delta={:+.9} quadratic_defect={quadratic_defect:+.9} defect_delta={:+.9}",
                quadratic_cp - cp,
                quadratic_defect - defect
            );
        }
    }
    Ok(())
}

#[test]
#[ignore = "fixed-boundary cutoff phase diagnostic; run in release mode"]
fn harris_subsonic_upstream_cutoff_phase() -> Result<(), String> {
    // Adjacent axial interval counts change both phase and spacing slightly,
    // but keep the boundary nodes and geometry fixed. Interpolate at the same
    // physical station; this does not independently isolate phase from dx.
    const EXTENSION: f64 = 0.5;
    const OUTER: f64 = 1.25;
    const CUTOFF: f64 = 36.0 / 38.240;
    // Keep even the central Cp derivative stencil entirely upstream.
    const STATION: f64 = 0.9;
    for axial_intervals in [128_usize, 129, 192, 193] {
        let dx = 2.0 / axial_intervals as f64;
        let nr = (OUTER / dx + 0.5).round() as usize;
        let dr = OUTER / (nr as f64 - 0.5);
        let source = (0..=axial_intervals)
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
            phi: vec![vec![0.0; nr]; axial_intervals + 1],
            axis_flux: source,
            dx,
            dr,
            mach: 0.9,
            gamma: 1.4,
        };
        let (iterations, residual) = grid
            .iterate(1e-6, 60000)
            .map_err(|value| format!("axial_intervals={axial_intervals}: residual={value}"))?;
        let index = (STATION + EXTENSION) / dx;
        let i = index.floor() as usize;
        assert!((i + 2) as f64 * dx - EXTENSION < CUTOFF);
        let weight = index - i as f64;
        let (radius, slope, _) = harris_geometry(STATION);
        // Evaluate both derivatives at the station's physical radius before
        // interpolating axially; no extrapolation in either direction.
        let cp = grid
            .surface_cp(i, radius, slope)
            .zip(grid.surface_cp(i + 1, radius, slope))
            .map(|(a, b)| (1.0 - weight) * a + weight * b);
        let defect = interpolated_radial_derivative(&grid.phi[i], dr, radius)
            .zip(interpolated_radial_derivative(&grid.phi[i + 1], dr, radius))
            .map(|(a, b)| (1.0 - weight) * a + weight * b - slope);
        let cutoff_index = (CUTOFF + EXTENSION) / dx;
        println!(
            "axial_intervals={axial_intervals} dx={dx:.9} dr={dr:.9} phase={:+.6} station_bracket=[{:.9},{:.9}] cp={cp:?} linear_defect={defect:?} iterations={iterations} residual={residual:.3e}",
            cutoff_index - cutoff_index.round(),
            i as f64 * dx - EXTENSION,
            (i + 1) as f64 * dx - EXTENSION
        );
    }
    Ok(())
}

#[test]
#[ignore = "fixed-boundary cutoff phase diagnostic; run in release mode"]
fn harris_subsonic_fixed_boundary_cutoff_phase() -> Result<(), String> {
    // Adjacent axial interval counts change both phase and spacing slightly,
    // but keep the boundary nodes and geometry fixed. Interpolate at the same
    // physical station; this does not independently isolate phase from dx.
    const EXTENSION: f64 = 0.5;
    const OUTER: f64 = 1.25;
    const CUTOFF: f64 = 36.0 / 38.240;
    const STATION: f64 = 0.9375;
    for axial_intervals in [128_usize, 129, 192, 193] {
        let dx = 2.0 / axial_intervals as f64;
        let nr = (OUTER / dx + 0.5).round() as usize;
        let dr = OUTER / (nr as f64 - 0.5);
        let source = (0..=axial_intervals)
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
            phi: vec![vec![0.0; nr]; axial_intervals + 1],
            axis_flux: source,
            dx,
            dr,
            mach: 0.9,
            gamma: 1.4,
        };
        let (iterations, residual) = grid
            .iterate(1e-6, 60000)
            .map_err(|value| format!("axial_intervals={axial_intervals}: residual={value}"))?;
        let index = (STATION + EXTENSION) / dx;
        let i = index.floor() as usize;
        let weight = index - i as f64;
        let (radius, slope, _) = harris_geometry(STATION);
        // Evaluate both derivatives at the station's physical radius before
        // interpolating axially; no extrapolation in either direction.
        let cp = grid
            .surface_cp(i, radius, slope)
            .zip(grid.surface_cp(i + 1, radius, slope))
            .map(|(a, b)| (1.0 - weight) * a + weight * b);
        let defect = interpolated_radial_derivative(&grid.phi[i], dr, radius)
            .zip(interpolated_radial_derivative(&grid.phi[i + 1], dr, radius))
            .map(|(a, b)| (1.0 - weight) * a + weight * b - slope);
        let cutoff_index = (CUTOFF + EXTENSION) / dx;
        println!(
            "axial_intervals={axial_intervals} dx={dx:.9} dr={dr:.9} phase={:+.6} station_bracket=[{:.9},{:.9}] cp={cp:?} linear_defect={defect:?} iterations={iterations} residual={residual:.3e}",
            cutoff_index - cutoff_index.round(),
            i as f64 * dx - EXTENSION,
            (i + 1) as f64 * dx - EXTENSION
        );
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

#[test]
#[ignore = "independent axial-boundary diagnostic; run in release mode"]
fn harris_subsonic_independent_axial_boundaries() -> Result<(), String> {
    const BODY_INTERVALS: usize = 32;
    const RADIAL_CELLS: usize = 97;
    const FIXED_EXTENSION: usize = 3 * BODY_INTERVALS;
    const MACH: f64 = 0.9;
    const GAMMA: f64 = 1.4;
    const TOLERANCE: f64 = 1e-6;
    const MAX_ITERATIONS: usize = 40000;
    const STATIONS: [f64; 5] = [0.25, 0.375, 0.5, 0.625, 0.75];
    let dx = 1.0 / BODY_INTERVALS as f64;
    for move_upstream in [true, false] {
        for moving_extension in [16_usize, 32, 64, 96] {
            let (upstream, downstream) = if move_upstream {
                (moving_extension, FIXED_EXTENSION)
            } else {
                (FIXED_EXTENSION, moving_extension)
            };
            let nx = upstream + BODY_INTERVALS + downstream + 1;
            let source = (0..nx)
                .map(|i| {
                    let x = (i as f64 - upstream as f64) * dx;
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
                dr: dx,
                mach: MACH,
                gamma: GAMMA,
            };
            let (iterations, residual) =
                grid.iterate(TOLERANCE, MAX_ITERATIONS).map_err(|value| {
                    format!("upstream={upstream} downstream={downstream}: residual={value}")
                })?;
            println!(
                "upstream={:.3} downstream={:.3} iterations={iterations} residual={residual:.3e}",
                upstream as f64 * dx,
                downstream as f64 * dx
            );
            for x in STATIONS {
                let i = upstream + (x / dx).round() as usize;
                let (radius, slope, _) = harris_geometry(x);
                let cp = grid
                    .surface_cp(i, radius, slope)
                    .ok_or_else(|| format!("unavailable Cp at x/l={x}"))?;
                println!(
                    "upstream={:.3} downstream={:.3} x/l={x:.3} cp={cp:.9}",
                    upstream as f64 * dx,
                    downstream as f64 * dx
                );
            }
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

#[test]
fn sonic_flux_consistent_across_mach_numbers() {
    const GAMMA: f64 = 1.4;
    // Eight rounding units relative to the evaluated flux scale, with an
    // absolute floor for the exact zero state.
    for mach in [0.0, 1e-7, 0.01, 0.3, 0.9, 1.0, 1.1] {
        for u in [-0.1_f64, 0.0, 0.001, 0.1] {
            let expected = axial_flux(u, mach, GAMMA);
            let actual = sonic_flux(u, u, mach, GAMMA);
            let tolerance = 8.0 * f64::EPSILON * expected.abs().max(1e-12);
            assert!(
                actual.is_finite() && (actual - expected).abs() <= tolerance,
                "mach={mach} u={u} actual={actual} expected={expected} tolerance={tolerance}"
            );
        }
    }
}

fn sonic_flux(left: f64, right: f64, mach: f64, gamma: f64) -> f64 {
    let coefficient = mach * mach * (gamma + 1.0);
    if coefficient == 0.0 {
        return axial_flux(right, mach, gamma);
    }
    let sonic = (1.0 - mach * mach) / coefficient;
    if left > sonic {
        if right < sonic {
            // Compute the loss directly instead of subtracting two nearly
            // equal fluxes around the peak.
            axial_flux(left, mach, gamma) - 0.5 * coefficient * (right - sonic).powi(2)
        } else {
            axial_flux(left, mach, gamma)
        }
    } else if right < sonic {
        axial_flux(right, mach, gamma)
    } else {
        axial_flux(sonic, mach, gamma)
    }
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
        self.iterate_with_forcing(tolerance, max_iterations, |_, _| 0.0)
    }

    fn iterate_with_forcing(
        &mut self,
        tolerance: f64,
        max_iterations: usize,
        forcing: impl Fn(usize, usize) -> f64,
    ) -> Result<(usize, f64), f64> {
        let nx = self.phi.len();
        let nr = self.phi[0].len();
        for iteration in 0..=max_iterations {
            let mut norm: f64 = 0.0;
            for i in 2..nx - 1 {
                for k in 0..nr - 1 {
                    let residual = self.residual(i, k) - forcing(i, k);
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
                    let residual = self.residual(i, k) - forcing(i, k);
                    let diagonal = self.diagonal(i, k);
                    if !diagonal.is_finite() || diagonal.abs() < 1e-14 {
                        return Err(diagonal);
                    }
                    let mut step = -residual / diagonal;
                    let mut accepted = false;
                    for _ in 0..20 {
                        self.phi[i][k] = old + step;
                        let trial = self.residual(i, k) - forcing(i, k);
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
#[ignore = "manufactured solution convergence diagnostic; run in release mode"]
fn manufactured_smooth_solution_iteration_convergence() -> Result<(), String> {
    // phi=A(x³+r⁴). The exact continuous forcing is
    // (1-M²-M²(gamma+1)phi_x)phi_xx + 16 A r².
    // Grid-dependent outer-centre and two left-row Dirichlet values are exact;
    // the axis flux is zero. No body geometry or physical validation is implied.
    const A: f64 = 0.01;
    const MACH: f64 = 0.3;
    const GAMMA: f64 = 1.4;
    const TOLERANCE: f64 = 1e-10;
    let exact = |x: f64, r: f64| A * (x.powi(3) + r.powi(4));
    let mut previous = f64::INFINITY;
    for n in [16_usize, 32, 64] {
        let h = 1.0 / n as f64;
        let mut grid = MixedGrid {
            phi: (0..=n)
                .map(|i| {
                    (0..=n)
                        .map(|k| {
                            if i < 2 || i == n || k == n {
                                exact(i as f64 * h, (k as f64 + 0.5) * h)
                            } else {
                                0.0
                            }
                        })
                        .collect()
                })
                .collect(),
            axis_flux: vec![0.0; n + 1],
            dx: h,
            dr: h,
            mach: MACH,
            gamma: GAMMA,
        };
        let forcing = |i: usize, k: usize| {
            let x = i as f64 * h;
            let r = (k as f64 + 0.5) * h;
            let lambda = 1.0 - MACH * MACH - MACH * MACH * (GAMMA + 1.0) * 3.0 * A * x * x;
            lambda * 6.0 * A * x + 16.0 * A * r * r
        };
        let (iterations, residual) = grid
            .iterate_with_forcing(TOLERANCE, 100_000, forcing)
            .map_err(|value| format!("n={n} residual={value}"))?;
        let mut error = 0.0_f64;
        for i in 2..n {
            for k in 0..n {
                error =
                    error.max((grid.phi[i][k] - exact(i as f64 * h, (k as f64 + 0.5) * h)).abs());
            }
        }
        assert!(
            error < previous / 2.0,
            "n={n} error={error} previous={previous}"
        );
        println!(
            "manufactured_solve n={n} dx={h:.9} max_error={error:.9e} iterations={iterations} residual={residual:.3e}"
        );
        previous = error;
    }
    Ok(())
}

#[test]
fn manufactured_axisymmetric_operator_and_integral_balance() {
    // phi=x^3+r^4, M=0: L(phi)=6x+16r², with zero axis
    // flux. The outer row and the first two axial rows are prescribed,
    // not solved. This tests operator truncation, not solution convergence.
    let mut previous_error = f64::INFINITY;
    for n in [16_usize, 32, 64, 128] {
        let h = 1.0 / n as f64;
        let nx = n + 1;
        let nr = n + 1;
        let grid = MixedGrid {
            phi: (0..nx)
                .map(|i| {
                    let x = i as f64 * h;
                    (0..nr)
                        .map(|k| {
                            let r = (k as f64 + 0.5) * h;
                            x.powi(3) + r.powi(4)
                        })
                        .collect()
                })
                .collect(),
            axis_flux: vec![0.0; nx],
            dx: h,
            dr: h,
            mach: 0.0,
            gamma: 1.4,
        };
        let mut max_error = 0.0_f64;
        for i in 2..nx - 1 {
            for k in 0..nr - 1 {
                let x = i as f64 * h;
                let r = (k as f64 + 0.5) * h;
                max_error = max_error.max((grid.residual(i, k) - (6.0 * x + 16.0 * r * r)).abs());
            }
        }
        assert!(max_error < previous_error / 3.9, "n={n} error={max_error}");
        println!("manufactured n={n} max_error={max_error:.9e}");
        previous_error = max_error;
    }

    // Independent boundary-flux summation for a nonzero, varying axis flux
    // and a general field: use precisely the rows updated by iterate().
    let nx = 19_usize;
    let nr = 13_usize;
    let dx = 0.08;
    let dr = 0.07;
    let grid = MixedGrid {
        phi: (0..nx)
            .map(|i| {
                (0..nr)
                    .map(|k| {
                        let x = i as f64 * dx;
                        let r = (k as f64 + 0.5) * dr;
                        0.03 * (x.sin() + x * r * r + r.cos())
                    })
                    .collect()
            })
            .collect(),
        axis_flux: (0..nx).map(|i| 0.002 * (i as f64 * dx).cos()).collect(),
        dx,
        dr,
        mach: 0.9,
        gamma: 1.4,
    };
    let mut volume_sum = 0.0;
    let mut boundary_sum = 0.0;
    for i in 2..nx - 1 {
        for k in 0..nr - 1 {
            let r = (k as f64 + 0.5) * dr;
            volume_sum += grid.residual(i, k) * r * dx * dr;
        }
        boundary_sum += dx
            * ((nr - 1) as f64 * dr * (grid.phi[i][nr - 1] - grid.phi[i][nr - 2]) / dr
                - grid.axis_flux[i]);
    }
    for k in 0..nr - 1 {
        let r = (k as f64 + 0.5) * dr;
        let slope = |j: usize| (grid.phi[j][k] - grid.phi[j - 1][k]) / dx;
        let inlet = sonic_flux(slope(1), slope(2), grid.mach, grid.gamma);
        let outlet = sonic_flux(slope(nx - 2), slope(nx - 1), grid.mach, grid.gamma);
        boundary_sum += r * dr * (outlet - inlet);
    }
    let difference = (volume_sum - boundary_sum).abs();
    assert!(difference < 1e-12 * (volume_sum.abs() + boundary_sum.abs()).max(1e-6));
    println!(
        "integral volume={volume_sum:.15e} boundary={boundary_sum:.15e} difference={difference:.3e}"
    );
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
