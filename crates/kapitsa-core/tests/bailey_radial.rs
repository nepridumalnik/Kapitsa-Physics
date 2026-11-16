//! Bailey TN D-6582 radial stencil checks, not a body-flow validation.
//! https://ntrs.nasa.gov/api/citations/19720004250/downloads/19720004250.pdf

fn axis(first: f64, second: f64, h: f64, flux: f64) -> Option<f64> {
    if ![first, second, h, flux].iter().all(|v| v.is_finite()) || h <= 0.0 {
        return None;
    }
    Some((second - first - flux) / (0.5 * h * h))
}

fn stretched_axis(first: f64, second: f64, h: f64, a: f64, b: f64, flux: f64) -> f64 {
    (2.0 * b - a * h).powi(3) / (4.0 * b * b * h * h) * ((b - a * h) * (second - first) - flux)
}

fn interior(values: [f64; 3], eta: f64, h: f64, a: f64, b: f64) -> f64 {
    let scale = (b - a * eta).powi(3) / (b * h * h);
    let base = (b - a * eta) / b;
    let skew = h / 2.0 * (1.0 / eta - 2.0 * a / b);
    scale * ((base - skew) * (values[0] - values[1]) + (base + skew) * (values[2] - values[1]))
}

#[test]
fn uniform_axis_and_line_source() {
    for h in [0.2_f64, 0.1, 0.05] {
        assert_eq!(axis(3.0, 3.0, h, 0.0), Some(0.0));
        assert!(
            (axis((h / 2.0).powi(2), (3.0 * h / 2.0).powi(2), h, 0.0).unwrap() - 4.0).abs() < 1e-12
        );
        let log_residual = axis((h / 2.0).ln(), (3.0 * h / 2.0).ln(), h, 1.0).unwrap();
        assert!((log_residual * h * h / 2.0 - (3.0_f64.ln() - 1.0)).abs() < 1e-12);
        assert!(
            (stretched_axis(0.2, 0.3, h, 0.0, 1.0, 0.02) - axis(0.2, 0.3, h, 0.02).unwrap()).abs()
                < 1e-12
        );
    }
    for h in [0.0, -1.0, f64::INFINITY, f64::NAN] {
        assert!(axis(0.0, 0.0, h, 0.0).is_none());
    }
}

#[test]
fn stretched_interior_second_order_away_from_axis() {
    for eta in [0.3_f64, 0.5] {
        assert_eq!(interior([4.0; 3], eta, 0.01, 0.4, 1.0), 0.0);
        let values = [(eta - 0.01).powi(2), eta.powi(2), (eta + 0.01).powi(2)];
        assert!((interior(values, eta, 0.01, 0.0, 1.0) - 4.0).abs() < 1e-10);
    }
    let (eta, a, b) = (0.4, 0.5, 1.0);
    let errors: Vec<_> = [0.04, 0.02, 0.01]
        .into_iter()
        .map(|h| {
            let values = [-1.0, 0.0, 1.0].map(|offset| {
                let z: f64 = eta + offset * h;
                (z / (b - a * z)).powi(2)
            });
            (interior(values, eta, h, a, b) - 4.0).abs()
        })
        .collect();
    assert!(errors[0] / errors[1] > 3.8 && errors[1] / errors[2] > 3.8);
}
