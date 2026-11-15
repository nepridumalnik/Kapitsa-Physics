"""Checks mixed residual assembly, not nonlinear convergence or validation."""

import unittest

from bailey_prototype import mixed_residual


class MixedResidualTests(unittest.TestCase):
    def test_smooth_subsonic_matches_analytic_operator(self):
        # phi = a*x^2 + b*r^2: axial flux divergence is exact for a
        # quadratic potential in the subsonic region; radial term is 4*b.
        a, b, mach, gamma = 0.005, 0.007, 0.7, 1.4
        for n in (16, 32, 64):
            h = 1 / n
            phi = [[a * (i*h)**2 + b * ((k + 0.5)*h)**2 for k in range(n)]
                   for i in range(n + 1)]
            i, k = n // 2, n // 4
            expected = (1 - mach**2 - mach**2*(gamma + 1)*2*a*i*h)*2*a + 4*b
            self.assertAlmostEqual(mixed_residual(phi, i, k, h, h, mach, gamma,
                                                  [0.0]*(n + 1)), expected, places=11)

    def test_compression_and_expansion_residuals(self):
        for n in (32, 64, 128):
            h = 1 / n
            for up, down, stationary in ((0.02, -0.02, True), (-0.02, 0.02, False)):
                slopes = [up] * (n // 2) + ([0.0] if stationary else []) + [down] * (n // 2)
                values = [0.0]
                for slope in slopes:
                    values.append(values[-1] + h*slope)
                phi = [[value] * 3 for value in values]
                source = [0.0] * len(phi)
                residuals = [mixed_residual(phi, i, 1, h, h, 1.0, 1.4, source)
                             for i in range(2, len(phi) - 1)]
                if stationary:
                    self.assertLess(max(map(abs, residuals)), 1e-12)
                else:
                    self.assertGreater(max(map(abs, residuals)), 1e-4)


if __name__ == "__main__":
    unittest.main()
