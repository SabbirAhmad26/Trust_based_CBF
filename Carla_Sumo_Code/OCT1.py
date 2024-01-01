from scipy.optimize import fsolve
import numpy as np



def OCT1(t0, v0, L):

    beta = .1
    ocpar = []
    # Define the system of equations as a function
    def equations(vars):
        a, b, c, d, tm = vars
        eq1 = 0.5*a*t0**2 + b*t0 + c - v0
        eq2 = 1/6*a*t0**3 + 0.5*b*t0**2 + c*t0 + d
        eq3 = a*tm + b
        eq4 = 1/6*a*tm**3 + 0.5*b*tm**2 + c*tm + d - L
        eq5 = beta - 0.5*b**2 + a*c
        return [eq1, eq2, eq3, eq4, eq5]

    # Initial guess for the solution
    initial_guess = [1.0, 1.0, 1.0, 1.0, 1.0]

    # Solve the system of equations using fsolve
    solutions = fsolve(equations, initial_guess)

    # Extract values from the solutions
    ai, bi, ci, di, tmi = solutions

    # Check conditions and calculate additional expression
    if np.isreal(tmi) and t0 + 0 < tmi <= t0 + 100:
        ocpar = [ai, bi, ci, di, tmi, 0.5*ai*tmi**2 + bi*tmi + ci]

    if not ocpar:
        ocpar = [0, 0.9, v0, 0, 0 ,0]

    return ocpar

