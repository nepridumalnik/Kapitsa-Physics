"""Check interpolation at physical coordinates independently of the solver."""

import unittest

from bailey_grid_diagnostic import sample


class SamplingTests(unittest.TestCase):
    def test_bilinear_field_on_cell_centered_radial_grid(self):
        for n in (9, 17, 33):
            h = 1 / (n - 1)
            phi = [[2 * i*h + 3 * (k + 0.5)*h for k in range(n)] for i in range(n)]
            self.assertAlmostEqual(sample(phi, 0.5, 0.125, h), 1.375)

    def test_outside_grid_rejected(self):
        with self.assertRaises(ValueError):
            sample([[0.0] * 4 for _ in range(4)], 0.1, 0.0, 0.1)


if __name__ == "__main__":
    unittest.main()
