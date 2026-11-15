"""Mixed-prototype refinement and outer-boundary diagnostic, not validation.

Run: python -B scripts/bailey_grid_diagnostic.py
The sinusoidal axis source is synthetic, not a matched experimental body.
"""

import math

from bailey_prototype import iterate_mixed


def sample(phi, x, r, h):
    """Bilinear interpolation at physical coordinates inside the grid."""
    xi = x / h
    ri = r / h - 0.5
    i, k = math.floor(xi), math.floor(ri)
    if i < 0 or i + 1 >= len(phi) or k < 0 or k + 1 >= len(phi[0]):
        raise ValueError("point outside interpolable cell centers")
    tx, tr = xi - i, ri - k
    return ((1 - tx) * ((1 - tr) * phi[i][k] + tr * phi[i][k + 1])
            + tx * ((1 - tr) * phi[i + 1][k] + tr * phi[i + 1][k + 1]))


def run(n, radial_extent):
    h = 1 / (n - 1)
    # The zero Dirichlet row is cell-centered; hold its physical radius fixed.
    nr = round(radial_extent / h + 0.5)
    actual_outer = (nr - 0.5) * h
    source = [0.01 * math.sin(2 * math.pi * i / (n - 1)) for i in range(n)]
    source[0] = source[-1] = 0.0
    phi = [[0.0] * nr for _ in range(n)]
    history = iterate_mixed(phi, source, h, h, 0.9, 1.4, 1e-6, 5000)
    x, radius = 0.5, 0.125
    potential = sample(phi, x, radius, h)
    velocity = (sample(phi, x + h, radius, h) - sample(phi, x - h, radius, h)) / (2 * h)
    return actual_outer, len(history) - 1, history[-1], potential, velocity


if __name__ == "__main__":
    for n, outer in ((17, 0.75), (33, 0.75), (17, 1.25), (33, 1.25)):
        r, iterations, residual, phi, u = run(n, outer)
        print(f"n={n} outer={r:.6f} iterations={iterations} residual={residual:.3g} "
              f"phi(0.5,0.125)={phi:.9g} u={u:.9g}")
