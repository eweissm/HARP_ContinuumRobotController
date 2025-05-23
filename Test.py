
import math
import numpy as np

from scipy.optimize import brentq
import sympy as sp


def CartesianToCurvature(X,BackboneLength):
        phi = math.atan2(X[1],X[0])

        r = np.sqrt(X[0]**2 + X[1]**2)

        def f(k):
            if k == 0:
                return np.inf  # avoid division by zero
            return (1/k) * (1 - np.cos(BackboneLength * k)) - r

        k_min = 1e-6
        k_max = np.pi / BackboneLength - 1e-6

        try:
            curvature = brentq(f, k_min, k_max)
        except ValueError:
            k= np.pi/s
            MaxR = (1/k) * (1 - np.cos(BackboneLength * k))
            print(f"Warning: Could not find solution for curvature. Note that max radius from origin is r={MaxR}, while current radius is {r}")
            curvature = None  # No root found in the interval

        return phi, curvature



s = .20
X=[.05,.05]

print(CartesianToCurvature(X,s))



# Define unknowns
l1, l2, l3 = sp.symbols('l1 l2 l3')
# Define known parameters
s, phi, kappa, d = sp.symbols('s phi kappa d', real=True, positive=True)

# Equation 17: mean length
eq1 = sp.Eq((l1 + l2 + l3)/3, s)

# Equation 18: phi as function of l1, l2, l3
phi_expr = sp.atan(sp.sqrt(3)*(l3 + l1 - 2*l2)/(3*(l2 - l3)))
eq2 = sp.Eq(phi, phi_expr)

# Equation 19: curvature
numerator = 2 * sp.sqrt((l1 - l2)**2 + (l2 - l3)**2 + (l3 - l1)**2)
denominator = d * (l1 + l2 + l3)
eq3 = sp.Eq(kappa, numerator / denominator)

# Solve the system for l1, l2, l3
solutions = sp.solve([eq1, eq2, eq3], (l1, l2, l3), dict=True)

# Display solutions
for sol in solutions:
    print("\nSolution:")
    for var, expr in sol.items():
        sp.pprint(sp.Eq(var, sp.simplify(expr)))