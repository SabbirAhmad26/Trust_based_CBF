import sympy as sp


def OCT1(t0, v0, L):
    # Define the variables
    beta = 0.1
    a, b, c, d, tm = sp.symbols('a b c d tm')

    # Define the equations
    eqns = [
        sp.Eq(0.5 * a * t0 ** 2 + b * t0 + c, v0),
        sp.Eq(sp.Rational(1, 6) * a * t0 ** 3 + 0.5 * b * t0 ** 2 + c * t0 + d, 0),
        sp.Eq(a * tm + b, 0),
        sp.Eq(sp.Rational(1, 6) * a * tm ** 3 + 0.5 * b * tm ** 2 + c * tm + d, L),
        sp.Eq(beta - 0.5 * b ** 2 + a * c, 0)
    ]

    eqns = [sp.simplify(eq) for eq in eqns]

    S = sp.nonlinsolve(eqns, [a, b, c, d, tm])
    S_list = list(S)

    ocpar = []

    # Iterate over the solutions
    for i in range(len(S_list)):
        if sp.im(S_list[i][4]) == 0 and t0 < sp.re(S_list[i][4]) <= t0 + 100:
            # Extract solution values
            ai = S_list[i][0].evalf()
            bi = S_list[i][1].evalf()
            ci = S_list[i][2].evalf()
            di = S_list[i][3].evalf()
            tmi = S_list[i][4].evalf()
            expr = 0.5 * ai * tmi ** 2 + bi * tmi + ci

            # Append to the result list
            ocpar.append(ai)
            ocpar.append(bi)
            ocpar.append(ci)
            ocpar.append(di)
            ocpar.append(tmi)
            ocpar.append(expr)

    #print(ocpar)

    return ocpar

ocpar = OCT1(5.5,7, 74)
a = 0
#OCT1(0, 0.1, 1.564)
# change the L value to the length of the road instead