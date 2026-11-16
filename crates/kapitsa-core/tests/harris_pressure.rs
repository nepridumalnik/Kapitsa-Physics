//! Table IV(a), Harris–Landrum. Diagnostic integration, not model validation.
//! Missing-point and endpoint assumptions are not experimental uncertainties.
//! https://ntrs.nasa.gov/api/citations/19660003947/downloads/19660003947.pdf

const DATA: &[(f64, f64)] = &[
    (0.011, 0.455),
    (0.017, 0.380),
    (0.024, 0.347),
    (0.038, 0.241),
    (0.056, 0.218),
    (0.084, 0.160),
    (0.111, 0.102),
    (0.139, 0.065),
    (0.167, 0.034),
    (0.194, 0.003),
    (0.250, -0.021),
    (0.306, -0.048),
    (0.361, -0.084),
    (0.417, -0.090),
    (0.472, -0.112),
    (0.528, -0.142),
    (0.584, -0.140),
    (0.667, -0.153),
    (0.722, -0.172),
    (0.833, -0.123),
    (0.889, 0.005),
    (0.917, 0.078),
    (0.944, 0.105),
    (0.972, 0.121),
    (0.983, 0.114),
    (0.994, 0.092),
];

// Table III(a), nominal A_base/A_max=0, l/d=7, Mach 0.90.
// The theoretical body is truncated for a sting; tunnel interference and
// measurement uncertainty have not been quantified here. Rounded x/l values
// are reported as printed and are not automatically grid-aligned.
#[path = "support/harris_data.rs"]
mod harris_data;
use harris_data::TABLE_III_A_MACH_090;

#[test]
fn measured_mach_090_stations_are_ordered_and_identified() {
    assert_eq!(TABLE_III_A_MACH_090.len(), 27);
    assert!(TABLE_III_A_MACH_090.windows(2).all(|p| p[0].0 < p[1].0));
    assert!(TABLE_III_A_MACH_090.contains(&(0.497, -0.135)));
    assert!(TABLE_III_A_MACH_090.contains(&(0.681, -0.094)));
}

fn weight(z: f64) -> f64 {
    let x = 2.0 * z - 1.0;
    -(4.24200 * x - 0.67736) * (1.0 - x * x).sqrt()
}

fn trapezoid(points: &[(f64, f64)]) -> f64 {
    points
        .windows(2)
        .map(|p| (p[1].0 - p[0].0) * (p[0].1 * weight(p[0].0) + p[1].1 * weight(p[1].0)) / 2.0)
        .sum()
}

fn linear_cp_integral(points: &[(f64, f64)]) -> f64 {
    const SUBDIVISIONS: usize = 100;
    let mut total = 0.0;
    for p in points.windows(2) {
        let dx = (p[1].0 - p[0].0) / SUBDIVISIONS as f64;
        for i in 0..SUBDIVISIONS {
            let t = (i as f64 + 0.5) / SUBDIVISIONS as f64;
            total += dx * (p[0].1 + t * (p[1].1 - p[0].1)) * weight(p[0].0 + t * (p[1].0 - p[0].0));
        }
    }
    total
}

#[test]
fn reproduce_pressure_integration_scenarios() {
    assert!((trapezoid(DATA) - 0.04638976).abs() < 1e-8);
    assert!((linear_cp_integral(DATA) - 0.04725367).abs() < 1e-8);
    for (cp, expected) in [(-0.172, 0.04915649), (-0.123, 0.04538477)] {
        let mut points = DATA.to_vec();
        points.push((0.778, cp));
        points.sort_by(|a, b| a.0.total_cmp(&b.0));
        assert!((linear_cp_integral(&points) - expected).abs() < 1e-8);
    }
    let mut extended = vec![(0.0, DATA[0].1)];
    extended.extend_from_slice(DATA);
    extended.push((1.0, DATA[DATA.len() - 1].1));
    assert!((linear_cp_integral(&extended) - 0.05044551).abs() < 1e-8);
}
