"""Conservative numerical flux for a 1D diagnostic of Bailey's TSD flux.

This Godunov flux is for a scalar pseudo-time conservation law, not Bailey's
full axisymmetric relaxation scheme. It does not implement the body solver.
The physical axial flux is equation (1b) of NASA TN D-6582:
https://ntrs.nasa.gov/api/citations/19720004250/downloads/19720004250.pdf
"""


def physical_flux(u, mach, gamma):
    return (1 - mach**2) * u - 0.5 * mach**2 * (gamma + 1) * u**2


def tsd_sonic_flux(left, right, mach, gamma):
    """Sonic-split spatial TSD flux; requires mach > 0 and gamma > 1."""
    if mach <= 0 or gamma <= 1:
        raise ValueError("expected positive Mach and gamma > 1")
    sonic = (1 - mach**2) / (mach**2 * (gamma + 1))
    peak = physical_flux(sonic, mach, gamma)
    upstream_loss = peak - physical_flux(left, mach, gamma) if left > sonic else 0
    downstream_loss = peak - physical_flux(right, mach, gamma) if right < sonic else 0
    return peak - upstream_loss - downstream_loss


def godunov_flux(left, right, mach, gamma):
    """Entropy flux for the strictly concave quadratic, mach > 0, gamma > 1."""
    if mach <= 0 or gamma <= 1:
        raise ValueError("expected positive Mach and gamma > 1")
    left_flux = physical_flux(left, mach, gamma)
    right_flux = physical_flux(right, mach, gamma)
    if left <= right:
        return min(left_flux, right_flux)
    sonic = (1 - mach**2) / (mach**2 * (gamma + 1))
    if right <= sonic <= left:
        return physical_flux(sonic, mach, gamma)
    return max(left_flux, right_flux)
