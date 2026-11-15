"""Verify the axis row of Bailey TN D-6582, equation (17).

https://ntrs.nasa.gov/api/citations/19720004250/downloads/19720004250.pdf
This is a numerical stencil check, not a transonic solver or validation.
Run: python -m unittest discover -s scripts -p test_bailey_axis.py -v
"""

import math
import unittest


def axis_radial_operator(phi_first, phi_second, radial_step, axis_flux):
    """Uniform radial cell centers at h/2 and 3h/2, with axis flux R*dR/dx."""
    if not all(math.isfinite(v) for v in (phi_first, phi_second, radial_step, axis_flux)):
        raise ValueError("non-finite stencil input")
    if radial_step <= 0:
        raise ValueError("radial step must be positive")
    outer_flux = (phi_second - phi_first)  # r_face/h = 1
    return (outer_flux - axis_flux) / (0.5 * radial_step**2)


class AxisStencilTests(unittest.TestCase):
    def test_constant_and_quadratic(self):
        for h in (0.2, 0.1, 0.05):
            self.assertEqual(axis_radial_operator(3.0, 3.0, h, 0.0), 0.0)
            self.assertAlmostEqual(axis_radial_operator((h / 2)**2, (3*h / 2)**2, h, 0.0), 4.0)

    def test_logarithmic_line_source(self):
        # Discrete flux uses a linear face gradient, so a logarithmic source
        # is consistent only in the refinement limit away from the axis.
        # At the first cell its relative error does not vanish under uniform
        # refinement: log(3) != 1. This test exposes that limitation.
        for h in (0.2, 0.1, 0.05):
            residual = axis_radial_operator(math.log(h / 2), math.log(3*h / 2), h, 1.0)
            self.assertAlmostEqual(residual * h**2 / 2, math.log(3) - 1.0)

    def test_invalid_inputs(self):
        for step in (0.0, -1.0, math.inf, math.nan):
            with self.assertRaises(ValueError):
                axis_radial_operator(0.0, 0.0, step, 0.0)


if __name__ == "__main__":
    unittest.main()
