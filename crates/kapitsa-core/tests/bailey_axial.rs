//! Smooth-field checks of Bailey TN D-6582 axial stencils, not shock validation.
//! https://ntrs.nasa.gov/api/citations/19720004250/downloads/19720004250.pdf

fn axial_term(v: [f64; 5], h: f64, mach: f64, gamma: f64, supersonic: bool) -> Option<f64> {
    if !v
        .iter()
        .chain([h, mach, gamma].iter())
        .all(|v| v.is_finite())
        || h <= 0.0
        || mach < 0.0
        || gamma <= 1.0
    {
        return None;
    }
    let [m3, m2, m1, center, plus] = v;
    let (first, second) = if supersonic {
        (
            (2.0 * center - m1 - 2.0 * m2 + m3) / (2.0 * h),
            (2.0 * center - 5.0 * m1 + 4.0 * m2 - m3) / h.powi(2),
        )
    } else {
        (
            (plus - m1) / (2.0 * h),
            (m1 - 2.0 * center + plus) / h.powi(2),
        )
    };
    Some((1.0 - mach.powi(2) - mach.powi(2) * (gamma + 1.0) * first) * second)
}

#[test]
fn axial_stencils_match_quadratic_and_converge_on_exponential() {
    let (h, x, mach, gamma) = (0.125, 0.5_f64, 0.9_f64, 1.4);
    let values = [-3.0, -2.0, -1.0, 0.0, 1.0].map(|i| (x + i * h).powi(2));
    let expected = (1.0 - mach.powi(2) - mach.powi(2) * (gamma + 1.0) * 2.0 * x) * 2.0;
    for branch in [false, true] {
        assert!((axial_term(values, h, mach, gamma, branch).unwrap() - expected).abs() < 1e-12);
        let x = 0.3_f64;
        let expected = (1.0 - mach.powi(2) - mach.powi(2) * (gamma + 1.0) * x.exp()) * x.exp();
        let errors: Vec<_> = [0.04, 0.02, 0.01]
            .into_iter()
            .map(|h| {
                let values = [-3.0, -2.0, -1.0, 0.0, 1.0].map(|i| (x + i * h).exp());
                (axial_term(values, h, mach, gamma, branch).unwrap() - expected).abs()
            })
            .collect();
        assert!(errors[0] / errors[1] > 3.5 && errors[1] / errors[2] > 3.5);
    }
}

#[test]
fn axial_stencil_rejects_invalid_inputs() {
    for (h, mach, gamma) in [
        (0.0, 0.9, 1.4),
        (1.0, -1.0, 1.4),
        (1.0, 0.9, 1.0),
        (f64::NAN, 1.0, 1.4),
    ] {
        assert!(axial_term([0.0; 5], h, mach, gamma, false).is_none());
    }
}
