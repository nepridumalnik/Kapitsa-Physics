"""Normal-shock jump diagnostics for Bailey's TSD equation (1b).

https://ntrs.nasa.gov/api/citations/19720004250/downloads/19720004250.pdf
This verifies flux algebra, not a discrete mixed-type solver or shock capture.
"""

import math
import unittest


def axial_flux(u, mach, gamma):
    """Conservative axial flux F(u), with u = dimensionless phi_x."""
    return (1.0 - mach**2) * u - 0.5 * mach**2 * (gamma + 1.0) * u**2


def sonic_perturbation(mach, gamma):
    return (1.0 - mach**2) / (mach**2 * (gamma + 1.0))


class NormalShockTests(unittest.TestCase):
    def test_flux_equality_and_character_change(self):
        # A stationary normal shock requires F(u1)=F(u2). For a
        # compressive branch, lambda changes from negative to positive.
        for mach in (0.9, 1.0, 1.1):
            gamma = 1.4
            sonic = sonic_perturbation(mach, gamma)
            u1, u2 = sonic + 0.02, sonic - 0.02
            self.assertAlmostEqual(axial_flux(u1, mach, gamma), axial_flux(u2, mach, gamma))
            coefficient = mach**2 * (gamma + 1)
            self.assertLess(1 - mach**2 - coefficient * u1, 0)
            self.assertGreater(1 - mach**2 - coefficient * u2, 0)

    def test_flux_difference_matches_jump_factorization(self):
        mach, gamma = 0.95, 1.4
        for u1, u2 in ((0.1, -0.05), (0.02, 0.07), (0.0, 0.0)):
            jump = axial_flux(u1, mach, gamma) - axial_flux(u2, mach, gamma)
            expected = (u1 - u2) * (1 - mach**2 - mach**2 * (gamma + 1) * (u1 + u2) / 2)
            self.assertTrue(math.isclose(jump, expected, rel_tol=1e-12, abs_tol=1e-15))


if __name__ == "__main__":
    unittest.main()
