"""One-dimensional TSD flux splitting diagnostic, not a complete solver.

The sonic mass-flux splitting is described in section 3.3.3 of Holst's
NASA review: https://ntrs.nasa.gov/api/citations/20020078395/downloads/20020078395.pdf
For Bailey's concave axial flux the loss from its sonic maximum is split into
upstream supersonic and downstream subsonic contributions. The original
full-potential derivation appears in NASA TM 85751, section 4:
https://ntrs.nasa.gov/api/citations/19840012417/downloads/19840012417.pdf
"""

import unittest

from bailey_flux import physical_flux


def tsd_flux(left, right, mach, gamma):
    sonic = (1 - mach**2) / (mach**2 * (gamma + 1))
    peak = physical_flux(sonic, mach, gamma)
    upstream_loss = peak - physical_flux(left, mach, gamma) if left > sonic else 0
    downstream_loss = peak - physical_flux(right, mach, gamma) if right < sonic else 0
    return peak - upstream_loss - downstream_loss


class SonicSplitTests(unittest.TestCase):
    def test_consistency_and_smooth_branch_selection(self):
        for u in (-0.1, 0, 0.1):
            self.assertAlmostEqual(tsd_flux(u, u, 1, 1.4), physical_flux(u, 1, 1.4))
        self.assertAlmostEqual(tsd_flux(0.1, 0.2, 1, 1.4), physical_flux(0.1, 1, 1.4))
        self.assertAlmostEqual(tsd_flux(-0.1, -0.2, 1, 1.4), physical_flux(-0.2, 1, 1.4))

    def test_expansion_not_stationary_and_shock_one_cell_profile(self):
        # The EO shock is spread over a link: its sonic intermediate state
        # balances both faces, unlike an abrupt two-state compression.
        for n in (32, 64, 128):
            for mach in (0.9, 1.0, 1.1):
                gamma = 1.4
                sonic = (1 - mach**2) / (mach**2 * (gamma + 1))
                up, down = sonic + 0.02, sonic - 0.02
                expected = physical_flux(up, mach, gamma)
                shock = [up] * (n // 2) + [sonic] + [down] * (n // 2)
                faces = [tsd_flux(a, b, mach, gamma) for a, b in zip(shock, shock[1:])]
                self.assertLess(max(abs(v - expected) for v in faces), 1e-14)
                expansion = [down] * (n // 2) + [up] * (n // 2)
                faces = [tsd_flux(a, b, mach, gamma) for a, b in zip(expansion, expansion[1:])]
                self.assertGreater(max(abs(v - expected) for v in faces), 1e-6)


if __name__ == "__main__":
    unittest.main()
