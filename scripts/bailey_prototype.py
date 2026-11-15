"""Research-only elliptic subset of Bailey's axisymmetric TSD equation.

Uniform cell-centred radial grid, finite zero-potential outer boundaries and
prescribed axis flux. This is NOT a transonic solver: it explicitly rejects
local hyperbolic flow and has no surface-pressure or shock reconstruction.
https://ntrs.nasa.gov/api/citations/19720004250/downloads/19720004250.pdf
"""

import math

from bailey_flux import tsd_sonic_flux


def mixed_residual(phi, i, k, dx, dr, mach, gamma, axis_flux):
    """Experimental mixed-type residual at an interior node, not an iteration.

    Requires i >= 2 and i+1 < nx; the caller supplies boundary values and
    must not treat the outer radial Dirichlet row as an interior row.
    """
    u0 = (phi[i - 1][k] - phi[i - 2][k]) / dx
    u1 = (phi[i][k] - phi[i - 1][k]) / dx
    u2 = (phi[i + 1][k] - phi[i][k]) / dx
    axial = (tsd_sonic_flux(u1, u2, mach, gamma)
             - tsd_sonic_flux(u0, u1, mach, gamma)) / dx
    r = (k + 0.5) * dr
    lower = k * dr / (r * dr**2)
    upper = (k + 1) * dr / (r * dr**2)
    p = phi[i][k]
    radial = (lower * (phi[i][k - 1] - p) if k else -axis_flux[i] / (r * dr))
    radial += upper * (phi[i][k + 1] - p)
    return axial + radial


class UnsupportedFlow(ValueError):
    pass


def iterate_mixed(phi, axis_flux, dx, dr, mach, gamma, tolerance, max_iterations):
    """Experimental damped coordinate Newton iteration; mutates phi in place.

    Two upstream columns, the downstream column and outer radial row are
    fixed Dirichlet data. No claim of convergence for arbitrary mixed flows.
    """
    nx = len(phi)
    nr = len(phi[0]) if nx else 0
    if (nx < 4 or nr < 2 or any(len(row) != nr for row in phi)
            or len(axis_flux) != nx or max_iterations < 1):
        raise ValueError("invalid grid")
    if not all(math.isfinite(v) for row in phi for v in row):
        raise ValueError("invalid initial field")
    if not all(math.isfinite(v) for v in (*axis_flux, dx, dr, mach, gamma, tolerance)):
        raise ValueError("non-finite input")
    if dx <= 0 or dr <= 0 or mach <= 0 or gamma <= 1 or tolerance <= 0:
        raise ValueError("invalid parameters")

    def norm():
        return max(abs(mixed_residual(phi, i, k, dx, dr, mach, gamma, axis_flux))
                   for i in range(2, nx - 1) for k in range(nr - 1))

    history = [norm()]
    for _ in range(max_iterations):
        if not math.isfinite(history[-1]):
            raise ArithmeticError("non-finite residual")
        if history[-1] <= tolerance:
            return history
        for i in range(2, nx - 1):
            for k in range(nr - 1):
                old = phi[i][k]
                f = mixed_residual(phi, i, k, dx, dr, mach, gamma, axis_flux)
                # Central numerical Jacobian also handles the C1 sonic split.
                delta = 1e-7 * max(1.0, abs(old))
                phi[i][k] = old + delta
                plus = mixed_residual(phi, i, k, dx, dr, mach, gamma, axis_flux)
                phi[i][k] = old - delta
                minus = mixed_residual(phi, i, k, dx, dr, mach, gamma, axis_flux)
                phi[i][k] = old
                derivative = (plus - minus) / (2 * delta)
                if not math.isfinite(derivative) or abs(derivative) < 1e-14:
                    raise ArithmeticError("singular local Jacobian")
                # Cap an update by local residual reduction, not a physical fit.
                step = -f / derivative
                for _ in range(20):
                    phi[i][k] = old + step
                    trial = mixed_residual(phi, i, k, dx, dr, mach, gamma, axis_flux)
                    if math.isfinite(trial) and abs(trial) <= abs(f):
                        break
                    step *= 0.5
                else:
                    phi[i][k] = old
        history.append(norm())
    if history[-1] <= tolerance:
        return history
    raise RuntimeError(f"mixed iteration failed: residual={history[-1]:.6g}")


def solve(axis_flux, nx, nr, dx, dr, mach, gamma, tolerance, max_iterations):
    """Return potential and residual history; raise if unsupported or unconverged.

    The source is supplied at each x node, including zero end boundaries.
    Point Gauss-Seidel is deliberately simple; this is not Bailey's SLOR.
    """
    if nx < 3 or nr < 2 or len(axis_flux) != nx or max_iterations < 1:
        raise ValueError("invalid grid or iteration limit")
    if not all(math.isfinite(v) for v in (*axis_flux, dx, dr, mach, gamma, tolerance)):
        raise ValueError("non-finite input")
    if dx <= 0 or dr <= 0 or not 0 <= mach < 1 or gamma <= 1 or tolerance <= 0:
        raise ValueError("unsupported inputs")
    if axis_flux[0] != 0 or axis_flux[-1] != 0:
        raise ValueError("axis source must vanish at axial boundaries")

    phi = [[0.0] * nr for _ in range(nx)]
    beta = 1 - mach * mach
    nonlinear = mach * mach * (gamma + 1)

    def coefficients(i, k):
        u = (phi[i + 1][k] - phi[i - 1][k]) / (2 * dx)
        lam = beta - nonlinear * u
        if not math.isfinite(lam) or lam <= 0:
            raise UnsupportedFlow("local sonic or supersonic flow")
        r = (k + 0.5) * dr
        lower = (k * dr) / (r * dr * dr)
        upper = ((k + 1) * dr) / (r * dr * dr)
        source = -axis_flux[i] / (r * dr) if k == 0 else 0.0
        return lam / dx**2, lower, upper, source

    def residual(i, k):
        axial, lower, upper, source = coefficients(i, k)
        p = phi[i][k]
        return (axial * (phi[i - 1][k] - 2 * p + phi[i + 1][k])
                + (lower * (phi[i][k - 1] - p) if k else 0.0)
                + upper * (phi[i][k + 1] - p) + source)

    history = []
    for _ in range(max_iterations):
        for i in range(1, nx - 1):
            for k in range(nr - 1):
                axial, lower, upper, _ = coefficients(i, k)
                phi[i][k] += residual(i, k) / (2 * axial + lower + upper)
        norm = max(abs(residual(i, k)) for i in range(1, nx - 1) for k in range(nr - 1))
        if not math.isfinite(norm):
            raise ArithmeticError("non-finite residual")
        history.append(norm)
        if norm <= tolerance:
            return phi, history
    raise RuntimeError(f"failed to converge: residual={history[-1]:.6g}")
