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

    def test_scalar_entropy_selection_rejects_physical_tsd_shock_orientation(self):
        # Bailey's physical normal shock has lambda<0 upstream and lambda>0
        # downstream: u decreases across the shock. The scalar pseudo-time
        # Godunov flux instead inserts the sonic flux for this orientation.
        # Consequently it must not be used as a spatial TSD shock scheme.
        for mach in (0.9, 1.0, 1.1):
            gamma = 1.4
            sonic = (1 - mach**2) / (mach**2 * (gamma + 1))
            upstream, downstream = sonic + 0.02, sonic - 0.02
            physical = physical_flux(upstream, mach, gamma)
            self.assertAlmostEqual(physical, physical_flux(downstream, mach, gamma))
            self.assertGreater(godunov_flux(upstream, downstream, mach, gamma), physical)
            for n in (32, 64, 128):
                states = [upstream] * (n // 2) + [downstream] * (n // 2)
                edges = [states[0], *states, states[-1]]
                fluxes = [godunov_flux(a, b, mach, gamma) for a, b in zip(edges, edges[1:])]
                residual = [fluxes[i + 1] - fluxes[i] for i in range(n)]
                self.assertGreater(max(map(abs, residual)), 0.0)
                self.assertAlmostEqual(sum(residual), 0.0)


if __name__ == "__main__":
    unittest.main()
