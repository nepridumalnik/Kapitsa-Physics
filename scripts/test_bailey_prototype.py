"""Numerical checks only; no experimental or shock validation."""

import math
import unittest

from bailey_prototype import UnsupportedFlow, solve


class PrototypeTests(unittest.TestCase):
    def test_zero_source(self):
        phi, history = solve([0.0] * 9, 9, 6, 0.25, 0.2, 0.8, 1.4, 1e-10, 10)
        self.assertEqual(history[-1], 0.0)
        self.assertTrue(all(value == 0.0 for row in phi for value in row))

    def test_source_residual_converges_and_boundaries_are_fixed(self):
        nx, nr = 17, 10
        flux = [0.001 * math.sin(2 * math.pi * i / (nx - 1)) for i in range(nx)]
        flux[0] = flux[-1] = 0.0
        phi, history = solve(flux, nx, nr, 0.1, 0.1, 0.6, 1.4, 1e-8, 10000)
        self.assertLessEqual(history[-1], 1e-8)
        self.assertLess(history[-1], history[0])
        self.assertTrue(all(v == 0.0 for v in phi[0] + phi[-1]))
        self.assertTrue(all(row[-1] == 0.0 for row in phi))
        self.assertGreater(max(abs(row[0]) for row in phi), 0.0)

    def test_rejects_nonconvergence_and_hyperbolic_flow(self):
        flux = [0.0, 1.0, -1.0, 0.0]
        with self.assertRaises(RuntimeError):
            solve(flux, 4, 4, 0.1, 0.1, 0.0, 1.4, 1e-20, 1)
        with self.assertRaises(UnsupportedFlow):
            solve(flux, 4, 4, 0.1, 0.1, 0.99, 1.4, 1e-12, 100)

    def test_grid_refinement_at_fixed_physical_location(self):
        # Interpolate between radial cell centers to compare at r=0.25;
        # this only checks an interior potential, not surface pressure.
        values = []
        for n in (17, 33, 65):
            h = 1 / (n - 1)
            flux = [0.001 * math.sin(2 * math.pi * i / (n - 1)) for i in range(n)]
            flux[0] = flux[-1] = 0.0
            phi, _ = solve(flux, n, n, h, h, 0.0, 1.4, 1e-7, 30000)
            k = n // 4
            values.append((phi[k][k - 1] + phi[k][k]) / 2)
        self.assertLess(abs(values[2] - values[1]), abs(values[1] - values[0]))


if __name__ == "__main__":
    unittest.main()
