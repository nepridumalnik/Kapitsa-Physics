"""Verify Bailey's stretched-grid radial stencils, equations (16) and (18).

https://ntrs.nasa.gov/api/citations/19720004250/downloads/19720004250.pdf
These smooth-field checks do not constitute a complete transonic solver.
Run: python -B -m unittest discover -s scripts -p test_bailey*.py -v
"""

import unittest

from test_bailey_axis import axis_radial_operator


def radius(eta, a, b):
    return eta / (b - a * eta)


def radial_interior(previous, center, following, eta, h, a, b):
    scale = (b - a * eta)**3 / (b * h**2)
    base = (b - a * eta) / b
    skew = h / 2 * (1 / eta - 2 * a / b)
    # Difference form preserves a constant field without cancellation.
    return scale * ((base - skew) * (previous - center)
                    + (base + skew) * (following - center))


def radial_axis(first, second, h, a, b, flux):
    common = (2 * b - a * h)**3 / (4 * b**2 * h**2)
    return common * ((b - a * h) * (second - first) - flux)


class RadialStencilTests(unittest.TestCase):
    def test_uniform_axis_matches_flux_form(self):
        for h in (0.2, 0.1, 0.05):
            self.assertAlmostEqual(radial_axis(0.2, 0.3, h, 0.0, 1.0, 0.02),
                                   axis_radial_operator(0.2, 0.3, h, 0.02))

    def test_interior_constant_and_uniform_quadratic(self):
        for eta in (0.3, 0.5):
            self.assertEqual(radial_interior(4, 4, 4, eta, 0.01, 0.4, 1.0), 0)
            self.assertAlmostEqual(radial_interior((eta - 0.01)**2, eta**2,
                                                   (eta + 0.01)**2, eta, 0.01, 0, 1), 4)

    def test_stretched_grid_second_order_away_from_axis(self):
        # For phi=r^2, the exact cylindrical radial Laplacian is 4.
        eta, a, b = 0.4, 0.5, 1.0
        errors = []
        for h in (0.04, 0.02, 0.01):
            values = [radius(eta + offset * h, a, b)**2 for offset in (-1, 0, 1)]
            errors.append(abs(radial_interior(*values, eta, h, a, b) - 4))
        self.assertGreater(errors[0] / errors[1], 3.8)
        self.assertGreater(errors[1] / errors[2], 3.8)


if __name__ == "__main__":
    unittest.main()
