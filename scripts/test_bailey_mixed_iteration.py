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

    def test_free_boundary_source_generates_mixed_flow(self):
        # No shock is prescribed at the outer boundary. These resolutions
        # demonstrate a mixed solution, not converged shock position.
        for n in (9, 17, 25):
            h = 1 / (n - 1)
            source = [0.01 * math.sin(2 * math.pi * i / (n - 1)) for i in range(n)]
            phi = [[0.0] * n for _ in range(n)]
            history = iterate_mixed(phi, source, h, h, 0.9, 1.4, 1e-6, 3000)
            sonic = (1 - 0.9**2) / (0.9**2 * 2.4)
            u = [(phi[i + 1][1] - phi[i - 1][1]) / (2*h) for i in range(2, n - 1)]
            crossings = [i for i in range(len(u) - 1) if u[i] > sonic >= u[i + 1]]
            self.assertLessEqual(history[-1], 1e-6)
            if n == 9:
                self.assertFalse(crossings)
            else:
                self.assertTrue(crossings)


if __name__ == "__main__":
    unittest.main()
