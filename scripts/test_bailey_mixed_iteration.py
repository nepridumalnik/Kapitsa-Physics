"""Research iteration checks; boundaries in the shock case prescribe a profile."""

import math
import unittest

from bailey_prototype import iterate_mixed


class MixedIterationTests(unittest.TestCase):
    def test_subsonic_source_converges(self):
        n = 17
        h = 1 / (n - 1)
        phi = [[0.0] * 9 for _ in range(n)]
        source = [0.0001 * math.sin(2 * math.pi * i / (n - 1)) for i in range(n)]
        history = iterate_mixed(phi, source, h, h, 0.7, 1.4, 1e-7, 3000)
        self.assertLessEqual(history[-1], 1e-7)
        self.assertLess(history[-1], history[0])

    def test_fixed_shock_boundary_profile(self):
        # Outer radial data impose the exact discontinuity; this verifies
        # iteration and residual refinement, not prediction of shock location.
        for n in (17, 33, 65):
            h = 1 / (n - 1)
            phi = [[0.02 * min(i*h, 0.5) - 0.02 * max(i*h - 0.5, 0)
                    for _ in range(4)] for i in range(n)]
            history = iterate_mixed(phi, [0.0] * n, h, h, 1.0, 1.4, 1e-8, 3000)
            self.assertLessEqual(history[-1], 1e-8)

    def test_iteration_limit_is_error(self):
        phi = [[0.0] * 5 for _ in range(9)]
        with self.assertRaises(RuntimeError):
            iterate_mixed(phi, [0, 0, 0.001, 0, 0, 0, 0, 0, 0],
                          0.1, 0.1, 0.7, 1.4, 1e-20, 1)


if __name__ == "__main__":
    unittest.main()
