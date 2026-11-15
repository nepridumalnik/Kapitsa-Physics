"""One-dimensional checks of Murman's conservative type-dependent operator.

Jameson, Transonic Flow Calculations, section 2.1, equations (18)-(23):
http://aero-comlab.stanford.edu/Papers/jameson_transonic_flow_calculations_VKI_1976.pdf
This is not an axisymmetric solver and does not enforce the entropy condition.
"""

import unittest

from bailey_flux import physical_flux


def residuals(slopes, h, mach, gamma):
    """Uniform-grid FCR operator for consecutive potential slopes.

    Switches use the average of the slopes on either side of each node.
    Return interior residuals and their shared modified face fluxes.
    """
    flux = [physical_flux(u, mach, gamma) for u in slopes]
    switches = [int(1 - mach**2 - mach**2 * (gamma + 1) * (a + b) / 2 < 0)
                for a, b in zip(slopes, slopes[1:])]
    modified = [flux[i + 1] - switches[i] * (flux[i + 1] - flux[i])
                for i in range(len(switches))]
    return [(b - a) / h for a, b in zip(modified, modified[1:])], modified


class MurmanSwitchTests(unittest.TestCase):
    def test_telescoping_conservation_at_type_transitions(self):
        slopes = [-0.1, -0.04, 0.08, 0.12, -0.03, -0.1]
        result, faces = residuals(slopes, 0.1, 1.0, 1.4)
        self.assertAlmostEqual(sum(result) * 0.1, faces[-1] - faces[0])

    def test_compression_and_expansion_both_admitted(self):
        # Physical compression runs from u>0 to u<0 at Mach 1. Murman's
        # conservative operator captures its flux jump but also admits the
        # reversed, entropy-violating discontinuity: conservation alone is
        # insufficient as an entropy selection criterion.
        for n in (32, 64, 128):
            for left, right in ((0.1, -0.1), (-0.1, 0.1)):
                slopes = [left] * (n // 2) + [right] * (n // 2)
                result, _ = residuals(slopes, 1 / n, 1.0, 1.4)
                self.assertLess(max(map(abs, result)), 1e-14)


if __name__ == "__main__":
    unittest.main()
