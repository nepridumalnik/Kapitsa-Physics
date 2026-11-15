"""One-dimensional conservative-flux checks, not a body-flow validation."""

import unittest

from bailey_flux import godunov_flux, physical_flux


class ConservativeFluxTests(unittest.TestCase):
    def test_smooth_consistency(self):
        for u in (-0.1, 0.0, 0.1):
            self.assertEqual(godunov_flux(u, u, 1.0, 1.4), physical_flux(u, 1.0, 1.4))

    def test_sonic_rarefaction_and_compressive_shock(self):
        mach, gamma = 1.0, 1.4
        sonic_flux = physical_flux(0.0, mach, gamma)
        self.assertEqual(godunov_flux(0.1, -0.1, mach, gamma), sonic_flux)
        self.assertEqual(godunov_flux(-0.1, 0.1, mach, gamma),
                         physical_flux(0.1, mach, gamma))

    def test_stationary_shock_and_global_conservation_on_grids(self):
        # F is concave. In pseudo-time its entropy shock has left < right;
        # this is not by itself a proof of the spatial TSD entropy condition.
        mach, gamma = 1.0, 1.4
        for n in (32, 64, 128):
            states = [-0.1] * (n // 2) + [0.1] * (n // 2)
            boundaries = [states[0], *states, states[-1]]
            fluxes = [godunov_flux(a, b, mach, gamma)
                      for a, b in zip(boundaries, boundaries[1:])]
            residual = [fluxes[i + 1] - fluxes[i] for i in range(n)]
            self.assertAlmostEqual(sum(residual), fluxes[-1] - fluxes[0])
            self.assertLess(max(map(abs, residual)), 1e-15)


if __name__ == "__main__":
    unittest.main()
