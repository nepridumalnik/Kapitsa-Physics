"""Experimental Bailey surface Cp reconstruction, no extrapolation.

Bailey NASA TN D-6582, equations (2) and (7):
https://ntrs.nasa.gov/api/citations/19720004250/downloads/19720004250.pdf
The caller must provide a converged field, radius and radius derivative. This
does not check slender-body applicability, shocks, or numerical convergence.
"""

import math


def surface_cp(phi, i, dx, dr, radius, slope):
    """Interpolate phi_x radially at fixed x, then apply Cp=-2 phi_x-R'^2.

    The axial derivative is taken before radial interpolation: differentiating
    potential sampled along a changing radius would instead give a total
    derivative. Linear interpolation is limited to interior radial centers.
    """
    nx = len(phi)
    nr = len(phi[0]) if nx else 0
    if nx < 3 or nr < 2 or any(len(row) != nr for row in phi) or not 1 <= i < nx - 1:
        raise ValueError("invalid field or axial index")
    if not all(math.isfinite(v) for v in (dx, dr, radius, slope)) or dx <= 0 or dr <= 0:
        raise ValueError("invalid geometry or grid")
    radial_index = radius / dr - 0.5
    if not 0 <= radial_index <= nr - 1:
        raise ValueError("surface lies outside radial cell centers")
    k = min(math.floor(radial_index), nr - 2)
    weight = radial_index - k
    low = (phi[i + 1][k] - phi[i - 1][k]) / (2 * dx)
    high = (phi[i + 1][k + 1] - phi[i - 1][k + 1]) / (2 * dx)
    cp = -2 * ((1 - weight) * low + weight * high) - slope**2
    if not math.isfinite(cp):
        raise ArithmeticError("non-finite pressure coefficient")
    return cp
