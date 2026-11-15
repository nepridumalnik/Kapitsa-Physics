"""Exploratory sting-mounted Sears–Haack geometry, NOT validation.

Harris & Landrum, zero-base-area theoretical shape, fineness 7, truncated at
36/38.240 of theoretical length for a sting:
https://ntrs.nasa.gov/api/citations/19660003947/downloads/19660003947.pdf
The finite box, axial boundaries, uniform radial grid, inviscid source and
constant-radius sting do not reproduce the experimental tunnel and mounting.
The axial domain ends at x/l=1 while the simplified sting continues there;
zero-potential axial boundaries are therefore not matched sting conditions.
The reported nominally zero-base shape is not the separate 0.532-base series.
"""

import math

from bailey_prototype import iterate_mixed
from bailey_surface_pressure import surface_cp


def geometry(x):
    """Return dimensionless radius, slope and axis flux R*R' before cutoff."""
    cutoff = 36 / 38.240
    z = min(x, cutoff)
    shape = max(0.0, 1 - (2*z - 1)**2)
    r = shape**0.75 / 14
    if x >= cutoff or z <= 0:
        return r, 0.0, 0.0
    derivative = -3 * (2*z - 1) * shape**(-0.25) / 14
    return r, derivative, r * derivative


def run(n, outer):
    h = 1 / (n - 1)
    nr = round(outer / h + 0.5)
    # Adjust dr independently of dx so the Dirichlet row has the same
    # physical radius on every grid, rather than rounding that radius.
    dr = outer / (nr - 0.5)
    source = [geometry(i*h)[2] for i in range(n)]
    phi = [[0.0] * nr for _ in range(n)]
    history = iterate_mixed(phi, source, h, dr, 0.9, 1.4, 1e-6, 10000)
    radius, slope, _ = geometry(0.5)
    cp = surface_cp(phi, (n - 1)//2, h, dr, radius, slope)
    return (nr - 0.5)*dr, len(history)-1, history[-1], cp


if __name__ == "__main__":
    for n, outer in ((17, 0.75), (33, 0.75), (65, 0.75),
                     (17, 1.25), (33, 1.25), (65, 1.25)):
        r, iterations, residual, cp = run(n, outer)
        print(f"n={n} outer={r:.6f} iterations={iterations} residual={residual:.3g} Cp(x/l=0.5)={cp:.8f}")
    print("Diagnostic only: tunnel, boundary layer, axial boundaries and sting unmatched.")
    print("Grid differences do not establish shock convergence or experimental validation.")
