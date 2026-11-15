#!/usr/bin/env python3
"""Diagnostic pressure integration, not a transonic model validation.

Harris & Landrum, table IV(a), Mach 1, l/d=7:
https://ntrs.nasa.gov/api/citations/19660003947/downloads/19660003947.pdf
The Cp entry at x/l=0.778 is blank. Endpoint and missing-point scenarios
below are illustrative assumptions, not bounds or measurement uncertainties.
"""

import math

DATA = (
    (0.011, 0.455), (0.017, 0.380), (0.024, 0.347), (0.038, 0.241),
    (0.056, 0.218), (0.084, 0.160), (0.111, 0.102), (0.139, 0.065),
    (0.167, 0.034), (0.194, 0.003), (0.250, -0.021), (0.306, -0.048),
    (0.361, -0.084), (0.417, -0.090), (0.472, -0.112), (0.528, -0.142),
    (0.584, -0.140), (0.667, -0.153), (0.722, -0.172), (0.833, -0.123),
    (0.889, 0.005), (0.917, 0.078), (0.944, 0.105), (0.972, 0.121),
    (0.983, 0.114), (0.994, 0.092),
)


def weight(z):
    return -(4.24200 * (2 * z - 1) - 0.67736) * math.sqrt(1 - (2 * z - 1) ** 2)


def trapezoid(points):
    return sum((b[0] - a[0]) * (a[1] * weight(a[0]) + b[1] * weight(b[0])) / 2
               for a, b in zip(points, points[1:]))


def linear_cp_integral(points, subdivisions=100):
    # Integrate interpolated Cp times the nonlinear geometry weight.
    total = 0.0
    for (x0, c0), (x1, c1) in zip(points, points[1:]):
        dx = (x1 - x0) / subdivisions
        for i in range(subdivisions):
            t = (i + 0.5) / subdivisions
            total += dx * (c0 + t * (c1 - c0)) * weight(x0 + t * (x1 - x0))
    return total


def main():
    print(f"measured interval, trapezoid on Cp*weight: {trapezoid(DATA):.8f}")
    print(f"measured interval, linear Cp midpoint integration: {linear_cp_integral(DATA):.8f}")
    # Two hypothetical values at the missing station, chosen as its neighboring
    # observations; these do not bound a shock or the true unmeasured Cp.
    for cp in (-0.172, -0.123):
        points = tuple(sorted((*DATA, (0.778, cp))))
        print(f"missing x/l=0.778 assumed Cp={cp:+.3f}: {linear_cp_integral(points):.8f}")
    # Constant endpoint extrapolation is one scenario, not a physical boundary condition.
    extended = ((0.0, DATA[0][1]), *DATA, (1.0, DATA[-1][1]))
    print(f"constant Cp endpoint extrapolation: {linear_cp_integral(extended):.8f}")
    print("No experimental uncertainty or independent model validation is established.")


if __name__ == "__main__":
    main()
