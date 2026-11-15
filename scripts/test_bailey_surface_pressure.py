"""Manufactured-field checks, not experimental pressure validation."""

import math
import unittest

from bailey_surface_pressure import surface_cp


class SurfacePressureTests(unittest.TestCase):
    def test_fixed_r_derivative_not_surface_total_derivative(self):
        # phi=x*r: phi_x=r, while d(phi(x,R(x)))/dx includes x*R'.
        h = 0.1
        phi = [[i*h*(k + 0.5)*h for k in range(8)] for i in range(9)]
        self.assertAlmostEqual(surface_cp(phi, 4, h, h, 0.25, 0.2), -0.54)

    def test_manufactured_smooth_field_second_order(self):
        errors = []
        for n in (16, 32, 64):
            h = 1 / n
            phi = [[math.sin(i*h) * math.exp((k + 0.5)*h)
                    for k in range(n)] for i in range(n + 1)]
            cp = surface_cp(phi, n//2, h, h, 0.25, 0.1)
            expected = -2 * math.cos(0.5) * math.exp(0.25) - 0.01
            errors.append(abs(cp - expected))
        self.assertGreater(errors[0] / errors[1], 3.8)
        self.assertGreater(errors[1] / errors[2], 3.8)

    def test_rejects_extrapolation_and_nonfinite_result(self):
        phi = [[0.0] * 4 for _ in range(5)]
        for radius in (0.0, 0.4, math.nan):
            with self.assertRaises(ValueError):
                surface_cp(phi, 2, 0.1, 0.1, radius, 0.0)
        phi[3][1] = math.inf
        with self.assertRaises(ArithmeticError):
            surface_cp(phi, 2, 0.1, 0.1, 0.15, 0.0)


if __name__ == "__main__":
    unittest.main()
