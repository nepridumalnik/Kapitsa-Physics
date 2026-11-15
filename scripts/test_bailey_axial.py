"""Check Bailey TN D-6582 uniform-grid axial stencils, equations (10)-(11).

https://ntrs.nasa.gov/api/citations/19720004250/downloads/19720004250.pdf
Local smooth-field verification only; it does not verify a shock or a solver.
Run: python -B -m unittest discover -s scripts -p test_bailey_axial.py -v
"""

import math
import unittest


def axial_term(values, h, mach, gamma, supersonic):
    """Evaluate lambda*phi_xx; values are phi[j-3] through phi[j+1].

    The caller supplies the local branch; choosing a branch consistently at
    sonic lines and shocks is a separate part of the nonlinear algorithm.
    """
    if len(values) != 5 or not all(math.isfinite(v) for v in (*values, h, mach, gamma)):
        raise ValueError("invalid stencil inputs")
    if h <= 0 or mach < 0 or gamma <= 1:
        raise ValueError("invalid physical inputs")
    m3, m2, m1, center, plus = values
    if supersonic:
        first = (2 * center - m1 - 2 * m2 + m3) / (2 * h)
        second = (2 * center - 5 * m1 + 4 * m2 - m3) / h**2
    else:
        first = (plus - m1) / (2 * h)
        second = (m1 - 2 * center + plus) / h**2
    return (1 - mach**2 - mach**2 * (gamma + 1) * first) * second


class AxialStencilTests(unittest.TestCase):
    def test_quadratic_exact_in_both_branches(self):
        h, x, mach, gamma = 0.125, 0.5, 0.9, 1.4
        values = [(x + i * h)**2 for i in range(-3, 2)]
        expected = (1 - mach**2 - mach**2 * (gamma + 1) * 2 * x) * 2
        for branch in (False, True):
            self.assertAlmostEqual(axial_term(values, h, mach, gamma, branch), expected)

    def test_smooth_field_second_order(self):
        x, mach, gamma = 0.3, 0.9, 1.4
        expected = (1 - mach**2 - mach**2 * (gamma + 1) * math.exp(x)) * math.exp(x)
        for branch in (False, True):
            errors = []
            for h in (0.04, 0.02, 0.01):
                values = [math.exp(x + i * h) for i in range(-3, 2)]
                errors.append(abs(axial_term(values, h, mach, gamma, branch) - expected))
            self.assertGreater(errors[0] / errors[1], 3.5)
            self.assertGreater(errors[1] / errors[2], 3.5)

    def test_invalid_inputs(self):
        for h, mach, gamma in ((0, 0.9, 1.4), (1, -1, 1.4), (1, 0.9, 1), (math.nan, 1, 1.4)):
            with self.assertRaises(ValueError):
                axial_term([0.0] * 5, h, mach, gamma, False)


if __name__ == "__main__":
    unittest.main()
