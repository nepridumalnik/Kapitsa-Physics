"""Geometry checks, not an experimental acceptance test."""

import unittest

from bailey_harris_diagnostic import geometry


class HarrisGeometryTests(unittest.TestCase):
    def test_maximum_radius_and_sting(self):
        r, slope, flux = geometry(0.5)
        self.assertAlmostEqual(r, 1 / 14)
        self.assertEqual(slope, 0)
        self.assertEqual(flux, 0)
        self.assertAlmostEqual(geometry(0.95)[0], geometry(1.0)[0])
        self.assertEqual(geometry(0.95)[2], 0)

    def test_flux_matches_radius_derivative(self):
        for x in (0.25, 0.75, 0.9):
            r, slope, flux = geometry(x)
            eps = 1e-6
            derivative = (geometry(x + eps)[0] - geometry(x - eps)[0]) / (2*eps)
            self.assertAlmostEqual(slope, derivative, places=8)
            self.assertAlmostEqual(flux, r * derivative, places=8)


if __name__ == "__main__":
    unittest.main()
