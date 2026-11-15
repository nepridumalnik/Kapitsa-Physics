//! Research-only elliptic prototype; neither a mixed-flow solver nor validation.

use std::f64::consts::PI;

#[derive(Debug, PartialEq)]
enum SolveError {
    UnsupportedFlow,
    Unconverged,
}

struct Grid {
    phi: Vec<Vec<f64>>,
    source: Vec<f64>,
    dx: f64,
    dr: f64,
    mach: f64,
    gamma: f64,
}

impl Grid {
    fn coefficients(&self, i: usize, k: usize) -> Result<(f64, f64, f64, f64), SolveError> {
        let u = (self.phi[i + 1][k] - self.phi[i - 1][k]) / (2.0 * self.dx);
        let lambda = 1.0 - self.mach.powi(2) - self.mach.powi(2) * (self.gamma + 1.0) * u;
        if !lambda.is_finite() || lambda <= 0.0 {
            return Err(SolveError::UnsupportedFlow);
        }
        let r = (k as f64 + 0.5) * self.dr;
        let lower = k as f64 * self.dr / (r * self.dr.powi(2));
        let upper = (k + 1) as f64 * self.dr / (r * self.dr.powi(2));
        let source = if k == 0 {
            -self.source[i] / (r * self.dr)
        } else {
            0.0
        };
        Ok((lambda / self.dx.powi(2), lower, upper, source))
    }

    fn residual(&self, i: usize, k: usize) -> Result<f64, SolveError> {
        let (axial, lower, upper, source) = self.coefficients(i, k)?;
        let p = self.phi[i][k];
        Ok(axial * (self.phi[i - 1][k] - 2.0 * p + self.phi[i + 1][k])
            + if k == 0 {
                0.0
            } else {
                lower * (self.phi[i][k - 1] - p)
            }
            + upper * (self.phi[i][k + 1] - p)
            + source)
    }

    fn solve(&mut self, tolerance: f64, limit: usize) -> Result<Vec<f64>, SolveError> {
        let (nx, nr) = (self.phi.len(), self.phi[0].len());
        let mut history = Vec::new();
        for _ in 0..limit {
            for i in 1..nx - 1 {
                for k in 0..nr - 1 {
                    let (axial, lower, upper, _) = self.coefficients(i, k)?;
                    self.phi[i][k] += self.residual(i, k)? / (2.0 * axial + lower + upper);
                }
            }
            let mut norm = 0.0_f64;
            for i in 1..nx - 1 {
                for k in 0..nr - 1 {
                    norm = norm.max(self.residual(i, k)?.abs());
                }
            }
            history.push(norm);
            if norm <= tolerance {
                return Ok(history);
            }
        }
        Err(SolveError::Unconverged)
    }
}

fn grid(nx: usize, nr: usize, dx: f64, dr: f64, mach: f64, source: Vec<f64>) -> Grid {
    Grid {
        phi: vec![vec![0.0; nr]; nx],
        source,
        dx,
        dr,
        mach,
        gamma: 1.4,
    }
}

fn sinusoidal_source(n: usize) -> Vec<f64> {
    let mut source: Vec<_> = (0..n)
        .map(|i| 0.001 * (2.0 * PI * i as f64 / (n - 1) as f64).sin())
        .collect();
    source[0] = 0.0;
    source[n - 1] = 0.0;
    source
}

#[test]
fn elliptic_iteration_and_rejection() {
    let mut zero = grid(9, 6, 0.25, 0.2, 0.8, vec![0.0; 9]);
    assert_eq!(zero.solve(1e-10, 10).unwrap(), vec![0.0]);
    assert!(zero.phi.iter().flatten().all(|v| *v == 0.0));
    let mut g = grid(17, 10, 0.1, 0.1, 0.6, sinusoidal_source(17));
    let history = g.solve(1e-8, 10000).unwrap();
    assert!(history[history.len() - 1] <= 1e-8 && history[history.len() - 1] < history[0]);
    assert!(g.phi[0].iter().chain(&g.phi[16]).all(|v| *v == 0.0));
    assert!(g.phi.iter().all(|row| row[9] == 0.0));
    assert!(g.phi.iter().any(|row| row[0] != 0.0));
    let source = vec![0.0, 1.0, -1.0, 0.0];
    assert_eq!(
        grid(4, 4, 0.1, 0.1, 0.0, source.clone()).solve(1e-20, 1),
        Err(SolveError::Unconverged)
    );
    assert_eq!(
        grid(4, 4, 0.1, 0.1, 0.99, source).solve(1e-12, 100),
        Err(SolveError::UnsupportedFlow)
    );
}

#[test]
fn elliptic_refinement_at_fixed_physical_point() {
    let mut values = Vec::new();
    for n in [17, 33, 65] {
        let h = 1.0 / (n - 1) as f64;
        let mut g = grid(n, n, h, h, 0.0, sinusoidal_source(n));
        g.solve(1e-7, 30000).unwrap();
        let k = n / 4;
        values.push((g.phi[k][k - 1] + g.phi[k][k]) / 2.0);
    }
    assert!((values[2] - values[1]).abs() < (values[1] - values[0]).abs());
}
