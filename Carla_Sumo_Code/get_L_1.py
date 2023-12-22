from sympy import symbols, solve


def get_L_1(t0, new, coe):
    varphi = 1.8
    if new[4] - coe[4] >= varphi:
        L1 = 400
    else:
        t1 = symbols('t1')
        s_new = (1 / 6) * new[0] * t1 ** 3 + (1 / 2) * new[1] * t1 ** 2 + new[2] * t1 + new[3]
        v_new = (1 / 2) * new[0] * t1 ** 2 + new[1] * t1 + new[2]
        s_coe = (1 / 6) * coe[0] * t1 ** 3 + (1 / 2) * coe[1] * t1 ** 2 + coe[2] * t1 + coe[3]

        # Solving the equation symbolically
        S = solve(s_coe - s_new - varphi * v_new, t1)
        ta = []
        for sol in S:
            real_part = sol.evalf()
            if sol.is_real and t0 <= real_part <= new[4]:
                ta.append(real_part)

        if not ta:
            L1 = 400
        else:
            ta = ta[0]
            L1 = (1 / 6) * new[0] * ta ** 3 + (1 / 2) * new[1] * ta ** 2 + new[2] * ta + new[3]

    return L1
