import numpy as np
import numpy as np
import time
import numpy as np
from scipy.optimize import linprog
from cvxopt import matrix, solvers, sqrt
import cvxopt
import cvxopt.solvers
from scipy.integrate import odeint
from cvxopt import matrix,sparse
import math
from scipy.optimize import minimize
from scipy.optimize import linprog
from scipy.optimize import fmin_slsqp
import numpy as np
#from scipy.optimize import odeint
from main_OCBF import second_order_model

global dt, u, cnt, noise1, noise2, const1, const2, const3, const4
global s1, s2, s3, initialvelocity

def update_table(car):
    size = len(car["order"])
    order = car["order"]
    car['table'] = []
    for j in range(size):
        # print(order[j],int(order[j]),car['que1'])
        temp = np.zeros(15)
        # 1st: index, 2-29: CP 1-28, 30: current lane
        # 31: order No, that is the index of the vehicle in the queue
        temp[0] = car['que1'][int(order[j])-1]['id'][1]
        for i in range(4, len(car['que1'][int(order[j])-1]['id'])):
            temp[car['que1'][int(order[j])-1]['id'][i]] = i - 3
        temp[13] = car['que1'][int(order[j])-1]['id'][2]
        temp[14] = int(order[j])
        car['table'].append(temp)
    return car

def  Event_detector(ego, que1, ip, ip_seen, index, CAV_e):

    # This function detects events for a CAV
    # Constants
    const1 = 0.1
    const2 = 0.2
    const3 = 0.3
    const4 = 0.4
    const1 = 0
    const2 = 0
    const3 = 0
    const4 = 0
    s1 = 3
    s2 = 3
    s3 = 0.7

    noise_term_position = max(abs(const1 * const2), abs(const1 * (1 - const2)))
    noise_term_speed = max(abs(const3 * const4), abs(const3 * (1 - const4)))

    flag_i = 0
    flag_i1 = 0
    flag_ip = 0
    flag_ip_seen = 0
    flag_i_trust = 0
    #CAV_e['x_tk'][id][0-6: 0: event for i, 1:event for ip, 2-5: event for ic, 6: event for ip_seen][othercar indicies]
    if ego["state"][1] >= CAV_e["v_tk"][ego["id"][1]][0][0] + s1 - noise_term_speed or \
            ego["state"][1] <= CAV_e["v_tk"][ego["id"][1]][0][0] - s1 + noise_term_speed or \
            ego["state"][0] >= CAV_e["x_tk"][ego["id"][1]][0][0] + s2 - noise_term_position or \
            ego["state"][0] <= CAV_e["x_tk"][ego["id"][1]][0][0] - s2 + noise_term_position:
        flag_i = 1

    # for k in range(len(ip)):
    #     if que1[k]["state"][1] >= CAV_e["v_tk"][ego["id"][1]][1][k] + s1 - noise_term_speed or \
    #             que1[k]["state"][1] <= CAV_e["v_tk"][ego["id"][1]][2][k] - s1 + noise_term_speed or \
    #             que1[k]["state"][0] >= CAV_e["x_tk"][ego["id"][1]][2][k] + s2 - noise_term_position or \
    #             que1[k]["state"][0] <= CAV_e["x_tk"][ego["id"][1]][2][k] - s2 + noise_term_position:
    #         flag_ip = 1
    #
    #     if que1[k]["trust"][0] >= que1[k]["trust"][1] + s3 or que1[k]["trust"][0] <= que1[k]["trust"][1] - s3:
    #         flag_i_trust = 1
    #
    for k in range(len(index)):
        for j in range(len(index[k])):
            if index[k][j] == -1:
                continue
            else:
                if que1[index[k][j]]["state"][1] >= CAV_e["v_tk"][ego["id"][1]][2 + k][j] + s1 - noise_term_speed or \
                        que1[index[k][j]]["state"][1] <= CAV_e["v_tk"][ego["id"][1]][2 + k][j] - s1 + noise_term_speed or \
                        que1[index[k][j]]["state"][0] >= CAV_e["x_tk"][ego["id"][1]][2 + k][j] + s2 - noise_term_position or \
                        que1[index[k][j]]["state"][0] <= CAV_e["x_tk"][ego["id"][1]][2 + k][j] - s2 + noise_term_position:
                    flag_i1 = 1
    #
    #             if que1[index[k][j]]["trust"][0] >= que1[index[k][j]]["trust"][1] + s3 or \
    #                     que1[index[k][j]]["trust"][0] <= que1[index[k][j]]["trust"][1] - s3:
    #                 flag_i_trust = 1
    #
    # if ip_seen:
    #     if que1[ip_seen]["state"][1] >= CAV_e["v_tk"][ego["id"][1]][8][0] + s1 - noise_term_speed or \
    #             que1[ip_seen]["state"][1] <= CAV_e["v_tk"][ego["id"][1]][8][0] - s1 + noise_term_speed or \
    #             que1[ip_seen]["state"][0] >= CAV_e["x_tk"][ego["id"][1]][8][0] + s2 - noise_term_position or \
    #             que1[ip_seen]["state"][0] <= CAV_e["x_tk"][ego["id"][1]][8][0] - s2 + noise_term_position:
    #         flag_ip_seen = 1

    return flag_i, flag_i1, flag_ip, flag_ip_seen, flag_i_trust

def OCBF_time(i, one, que, ip, index, position):
    vmax = 15
    dt = 0.1
    if one['state'][0] > one["metric"][-1]:
        v = 10
        u1 = 0
        x = one['state'][0] + v * dt
        rt = [x, v, u1]
        return rt

    # noise1 = const1 * (np.random.rand() - const2)
    # noise2 = const3 * (np.random.rand() - const4)

    x0 = one['state']
    c = np.array(one['ocpar'])
    t = dt * i
    eps = 10
    psc = 0.1

    phiRearEnd = one['phiRearEnd']
    deltaSafetyDistance = one['carlength']

    umax = 4
    umin = -6

    A = np.array([[1, 0], [-1, 0]])
    b = np.array([umax, -umin])

    # if one['id'][1] != 1:
    # vd = max(0.5 * c[0] * t**2 + c[1] * t + c[2], vmax)
    vd = 0.5 * c[0] * t**2 + c[1] * t + c[2]
    u_ref = max(c[0] * t + c[1], 0)
    print(u_ref,vd)
    # else:
    # if one["id"][1] == 1:
    #     vd = 4
    #     u_ref = -6
    # else:
    #     vd = 10
    #     u_ref = 3

    # CLF
    phi0 = -eps * (x0[1] - vd) ** 2
    phi1 = 2 * (x0[1] - vd)
    A = np.append(A, [[phi1, -1]], axis=0)
    b = np.append(b, [phi0])


    for k in ip:
        k_rear = one['k_rear']
        s0 = que[int(k)]['state'][0]
        v0 = que[int(k)]['state'][1]

        LfB = v0 - x0[1] + k_rear * (s0 - x0[0] - phiRearEnd * x0[1] - deltaSafetyDistance)
        LgB = phiRearEnd

        A = np.append(A, [[LgB, 0]], axis=0)
        b = np.append(b, [LfB])

    for k in range(len(index)):
        for j in range(len(index[k])):
            if index[k][j] == -1:
                continue
            else:
                d1 = que[index[k][j]-1]['metric'][position[k][j] + 3] - que[index[k][j]-1]['state'][0] #6
                d2 = one['metric'][k + 4] - x0[0] #4

            phiLateral = one['phiLateral']
            k_lateral = one['k_lateral'][k]
            L = one['metric'][k + 4]
            bigPhi = phiLateral * x0[0] / L

            h = d2 - d1 - bigPhi * x0[1] - deltaSafetyDistance
            LgB = bigPhi
            LfB = que[index[k][j]]['state'][1] - x0[1] - phiLateral * x0[1]**2 / L + k_lateral*h
            if LgB != 0:
                A = np.append(A, [[LgB, 0]], axis=0)
                b = np.append(b, [LfB])


    vmin = 0

    b_vmax = vmax - x0[1]
    b_vmin = x0[1] - vmin


    A = np.append(A, [[1.0, 0.0]], axis=0)
    b = np.append(b, [b_vmax])

    A = np.append(A, [[-1.0, 0.0]], axis=0)
    b = np.append(b, [b_vmin])


    H = np.array([[1.00e+00, 0.00e+00],
                  [0.00e+00, psc]])
    f = np.array([[-u_ref],
                  [0.00e+00]])

    # Create matrices for cvxopt.solvers.qp
    P = matrix(H)
    q = matrix(f)
    G = matrix(A)  # cvxopt uses <= inequality, so multiply by -1
    h = matrix(b)

    try :
        options = {'show_progress': False}
        Solution = cvxopt.solvers.qp(P, q, G, h, options=options)
        if Solution["status"] == 'optimal':
            u = Solution['x']
        else:
            u = Solution['x']
            if u[0] > umax:
                u[0] = umax
            else:
                u[0] = umin

    except:
        # Handle the case when no optimal solution is found
        u = matrix([umin, 0.0])  # [-cd * m * g, 0]

    a = (u[0], 0, 0)
    t = [0, 0.1]
    y0 = [x0[0], x0[1]]

    result = odeint(second_order_model, y0, t, args=a)
    rt = [result[-1,0], result[-1,1], a[0]]
    return rt

def OCBF_event(i, one, que, ip, index, position, flags):

    infeasiblity = 0
    vmax = 15
    dt = 0.1
    s1 = 3
    s2 = 3
    s3 = 0.7
    # noise1 = const1 * (np.random.rand() - const2)
    # noise2 = const3 * (np.random.rand() - const4)
    x0 = one['state']
    c = np.array(one['ocpar'])
    t = dt * i
    eps = 10
    psc = 0.1

    if one['state'][0] > one["metric"][-1]:
        v = 10
        u1 = 0
        x = one['state'][0] + v * dt
        rt = [x, v, u1]
        return rt, 0

    elif 1 in flags:
        deltaSafetyDistance = one["carlength"]
        # physical limitations on control inputs
        umax = 4
        umin = -6
        vmin = 0
        A = np.array([[1, 0], [-1, 0]])
        b = np.array([umax, -umin])

        # reference trajectory
        vd = 0.5 * c[0] * t**2 + c[1] * t + c[2]
        u_ref = max(c[0] * t + c[1], 0)


        # CLF
        phi0 = -eps * (x0[1] - vd) ** 2
        phi1 = 2 * (x0[1] - vd)
        A = np.append(A, [[phi1, -1]], axis=0)
        b = np.append(b, [phi0])

        # for k in range(len(ip)):
        #     # Extracting values
        #     k_rear = one['k_rear']
        #     phiRearEnd = one['phiRearEnd']
        #
        #     v_tk, x_tk = x0[1], x0[0]
        #     vp_tk, xp_tk = que[ip[k]]['state'][1], que[ip[k]]['state'][0]
        #
        #     C1_a = [phiRearEnd, +1, 0, -1]
        #     C1_b = -deltaSafetyDistance
        #     v_a = np.array([[1, 0, 0, 0], [-1, 0, 0, 0]])
        #     v_b = np.array([v_tk + s1, s1 - v_tk])
        #     x_a = np.array([[0, 1, 0, 0], [0, -1, 0, 0]])
        #     x_b = np.array([x_tk + s2, s2 - x_tk])
        #     vp_a = np.array([[0, 0, 1, 0], [0, 0, -1, 0]])
        #     vp_b = np.array([vp_tk + s1, s1 - vp_tk])
        #     xp_a = np.array([[0, 0, 0, 1], [0, 0, 0, -1]])
        #     xp_b = np.array([xp_tk + s2, s2 - xp_tk])
        #
        #     A_lin = np.vstack((C1_a, v_a, x_a, vp_a, xp_a))
        #     b_lin = np.hstack((C1_b, v_b, x_b, vp_b, xp_b))
        #     f_lin = np.array([-k_rear * phiRearEnd - 1, -k_rear * 1, 1, k_rear * 1])
        #
        #     options = {'disp': False}
        #     result = linprog(f_lin, A_ub=A_lin, b_ub=b_lin, options=options)
        #
        #     if result.success:
        #         Lf_terms = result.fun - k_rear * deltaSafetyDistance
        #         Lg_term = phiRearEnd
        #     else:
        #         Lg_term = phiRearEnd
        #         Lf_terms = vp_tk - v_tk + k_rear * (xp_tk - x_tk - phiRearEnd * v_tk - deltaSafetyDistance)
        #
        #     A = np.append(A, [[Lg_term, 0]], axis=0)
        #     b = np.append(b, [Lf_terms])
        #
        # for k in range(len(one['see'])):
        #     # if numel(intersect(one.see(k), ip))
        #     if que[one['see'][k]]['id'][2] == one['id'][2] and que[one['see'][k]]['state'][0] >= one['state'][0]:
        #         k_rear = one['k_rear']
        #         phiRearEnd = one['phiRearEnd']
        #         v_tk = x0[1]
        #         x_tk = x0[0]
        #         xp_tk = que[one['see'][k]]['state'][0]  # position of cav ip
        #         vp_tk = que[one['see'][k]]['state'][1]  # velocity of cav ip
        #         C1_a = [phiRearEnd, +1, 0, -1]
        #         C1_b = -deltaSafetyDistance
        #         v_a = np.array([[1, 0, 0, 0], [-1, 0, 0, 0]])
        #         v_b = np.array([v_tk + s1, s1 - v_tk])
        #         x_a = np.array([[0, 1, 0, 0], [0, -1, 0, 0]])
        #         x_b = np.array([x_tk + s2, s2 - x_tk])
        #         vp_a = np.array([[0, 0, 1, 0], [0, 0, -1, 0]])
        #         vp_b = np.array([vp_tk + s1, s1 - vp_tk])
        #         xp_a = np.array([[0, 0, 0, 1], [0, 0, 0, -1]])
        #         xp_b = np.array([xp_tk + s2, s2 - xp_tk])
        #
        #         A_lin = np.vstack((C1_a, v_a, x_a, vp_a, xp_a))
        #         b_lin = np.hstack((C1_b, v_b, x_b, vp_b, xp_b))
        #         f_lin = np.array([-k_rear * phiRearEnd - 1, -k_rear * 1, 1, k_rear * 1])
        #         options = {'disp': False}
        #         result = linprog(f_lin, A_ub=A_lin, b_ub=b_lin, options=options)
        #
        #         if result.success:
        #             Lf_terms = result.fun - k_rear * deltaSafetyDistance
        #             Lg_term = phiRearEnd
        #         else:
        #             Lg_term = phiRearEnd
        #             Lf_terms = vp_tk - v_tk + k_rear * (xp_tk - x_tk - phiRearEnd * v_tk - deltaSafetyDistance)
        #
        #         # Assuming A and b are defined before this code snippet
        #         A = np.append(A, [[Lg_term, 0]], axis=0)
        #         b = np.append(b, [Lf_terms])
        #
        for k in range(len(index)):
            for j in range(len(index[k])):
                if index[k][j] == -1:
                    continue
                else:
                    v_tk = x0[1]
                    x_tk = x0[0]
                    index_1 = index[k][j] - 1
                    position_1 = position[k][j]
                    vl_tk = que[index_1]['state'][1]
                    xl_tk = que[index_1]['state'][0]
                    d1 = que[index_1]['metric'][position[k][j] + 3] - que[index[k][j] - 1]['state'][0]  # 6
                    d2 = one['metric'][k + 4] - x0[0]  # 4

                k_lateral = one['k_lateral'][k]
                phiLateral = one['phiLateral']
                L = one['metric'][k + 4]
                bigPhi = phiLateral * x0[0] / L
                x_init = [v_tk, x_tk, vl_tk, xl_tk]
                lb = [v_tk - s1, x_tk - s2, vl_tk - s1, xl_tk - s2]
                ub = [v_tk + s1, x_tk + s2, vl_tk + s1, xl_tk + s2]

                def fun(x):
                    return x[2] - x[0] - phiLateral / L * x[0]**2 + k_lateral * (
                        (one['metric'][k + 4] - x[1]) - (que[index_1]['metric'][position_1+ 3] - x[3])
                        - phiLateral / L * x[2] * x[0] - deltaSafetyDistance)

                def constraint(x):
                    return -(one['metric'][k + 4] - x[1]) + (que[index_1]['metric'][position_1 + 3] - x[3]) + \
                           phiLateral / L * x[2] * x[0] + deltaSafetyDistance

                def nonlinfcn(x):
                    return constraint(x), 0

                Aeq = []
                beq = []
                A_quad = []
                b_quad = []
                # lb = [v_tk - s1, x_tk - s2, vl_tk - s1, xl_tk - s2]
                # ub = [v_tk + s1, x_tk + s2, vl_tk + s1, xl_tk + s2]
                lb = [max(v_tk - s1, vmin), max(x_tk - s2, 0), max(vl_tk - s1, vmin), max(xl_tk - s2, 0)]
                ub = [min(v_tk + s1,vmax), x_tk + s2, min(vl_tk + s1,vmax), xl_tk + s2]

                # index_1 = index[k][j]
                # position_1 = position[k][j]
                rt_slack = OCBF_time(i, one, que, ip, index, position)

                result = fmin_slsqp(fun, x_init, bounds=list(zip(lb, ub)), f_eqcons=nonlinfcn)
                fval_quad = result[2]  # Assuming the third element of the result corresponds to the cost value

                if rt_slack[2] >= 0:
                    Lf_terms = fval_quad  # Assuming the third element of the result corresponds to the cost value
                    Lg_term = phiLateral / L * max((x_tk - s2), 0.01)

                else:
                    Lf_terms = fval_quad # Assuming the third element of the result corresponds to the cost value
                    Lg_term = phiLateral / L * (x_tk + s2)

                A = np.append(A, [[Lg_term, 0]], axis=0)
                b = np.append(b, [Lf_terms])



        b_vmax = vmax - x0[1]
        b_vmin = x0[1] - vmin
        A = np.append(A, [[1.0, 0.0]], axis=0)
        b = np.append(b, [b_vmax])

        A = np.append(A, [[-1.0, 0.0]], axis=0)
        b = np.append(b, [b_vmin])

        H = np.array([[1.00e+00, 0.00e+00],
                      [0.00e+00, psc]])
        f = np.array([[-u_ref], [0.00e+00]])

        # Create matrices for cvxopt.solvers.qp
        P = matrix(H)
        q = matrix(f)
        G = matrix(A)  # cvxopt uses <= inequality, so multiply by -1
        h = matrix(b)

        try:
            options = {'show_progress': False}
            Solution = cvxopt.solvers.qp(P, q, G, h, options=options)
            if Solution["status"] == 'optimal':
                u = Solution['x']
            else:
                infeasiblity = 1
                u = Solution['x']
                if u[0] > umax:
                    u[0] = umax
                else:
                    u[0] = umin

        except:
            # Handle the case when no optimal solution is found
            infeasiblity = 1
            u = matrix([umin, 0.0])  # [-cd * m * g, 0]

        a = (u[0], 0, 0)
        t = [0, 0.1]
        y0 = [x0[0], x0[1]]

        result = odeint(second_order_model, y0, t, args=a)
        rt = [result[-1, 0], result[-1, 1], a[0]]


        if rt[1] < 0:
            rt[1] = 0

        if rt[0] < 0:
            time.sleep(1)

    else:
        u = x0[2]
        a = (u, 0, 0)
        t = [0, dt]
        y0 = [x0[0], x0[1]]

        result = odeint(second_order_model, y0, t, args=a)
        rt = [result[-1,0], result[-1,1], a[0]]
        if rt[1] < 0:
            rt[1] = 0
        infeasiblity = 0

    return rt, infeasiblity