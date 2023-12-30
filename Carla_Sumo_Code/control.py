import numpy as np
import time
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
import random
random.seed(5)
#from scipy.optimize import odeint
from main_OCBF import second_order_model

from metavariable import s1, s2, s3, dt, noise1, noise2, const1, const2, const3, const4

def update_table(car):
    size = len(car["order"])
    order = car["order"]
    car['table'] = []
    for j in range(size):
        # print(order[j],int(order[j]),car['que'])
        temp = np.zeros((15),dtype=int)
        # 1st: index, 2-29: CP 1-28, 30: current lane
        # 31: order No, that is the index of the vehicle in the queue
        temp[0] = car['que'][order[j]-1]['id'][1]
        for i in range(4, len(car['que'][order[j]-1]['id'])):
            temp[car['que'][order[j]-1]['id'][i]] = i - 3
        temp[13] = car['que'][order[j]-1]['id'][2]
        temp[14] = order[j]
        car['table'].append(temp)
    return car

def  Event_detector(ego, que, ip, index, CAV_e):

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

    for k in range(len(ip)):
        ip_index = ip[k] - 1
        if que[ip_index]["state"][1] >= CAV_e["v_tk"][ego["id"][1]][1][k] + s1 - noise_term_speed or \
                que[ip_index]["state"][1] <= CAV_e["v_tk"][ego["id"][1]][1][k] - s1 + noise_term_speed or \
                que[ip_index]["state"][0] >= CAV_e["x_tk"][ego["id"][1]][1][k] + s2 - noise_term_position or \
                que[ip_index]["state"][0] <= CAV_e["x_tk"][ego["id"][1]][1][k] - s2 + noise_term_position:
            flag_ip = 1

        # if que[k]["trust"][0] >= que[k]["trust"][1] + s3 or que[k]["trust"][0] <= que[k]["trust"][1] - s3:
        #     flag_i_trust = 1
    #
    for k in range(len(index)):
        for j in range(len(index[k])):
            if index[k][j] == -1:
                continue
            else:
                ic_index = index[k][j]-1
                if que[ic_index]["state"][1] >= CAV_e["v_tk"][ego["id"][1]][2 + k][j] + s1 - noise_term_speed or \
                        que[ic_index]["state"][1] <= CAV_e["v_tk"][ego["id"][1]][2 + k][j] - s1 + noise_term_speed or \
                        que[ic_index]["state"][0] >= CAV_e["x_tk"][ego["id"][1]][2 + k][j] + s2 - noise_term_position or \
                        que[ic_index]["state"][0] <= CAV_e["x_tk"][ego["id"][1]][2 + k][j] - s2 + noise_term_position:
                    flag_i1 = 1
    #
    #             if que[index[k][j]]["trust"][0] >= que[index[k][j]]["trust"][1] + s3 or \
    #                     que[index[k][j]]["trust"][0] <= que[index[k][j]]["trust"][1] - s3:
    #                 flag_i_trust = 1
    #
    for k in range(len(ego['see'])):
        if que[ego['see'][k]]['lane'] == ego['lane']:
            ip_seen_index = ego['see'][k]
            if que[ip_seen_index]["state"][1] >= CAV_e["v_tk"][ego["id"][1]][6][k] + s1 - noise_term_speed or \
                    que[ip_seen_index]["state"][1] <= CAV_e["v_tk"][ego["id"][1]][6][k] - s1 + noise_term_speed or \
                    que[ip_seen_index]["state"][0] >= CAV_e["x_tk"][ego["id"][1]][6][k] + s2 - noise_term_position or \
                    que[ip_seen_index]["state"][0] <= CAV_e["x_tk"][ego["id"][1]][6][k] - s2 + noise_term_position:
                flag_ip_seen = 1

    return flag_i, flag_i1, flag_ip, flag_ip_seen, flag_i_trust

def OCBF_time(i, ego, que):

    vmax = 20
    dt = 0.1
    infeasiblity = 0
    if ego['state'][0] > ego["metric"][-1]:
        v = 10
        u1 = 0
        x = ego['state'][0] + v * dt
        rt = [x, v, u1]
        return rt, infeasiblity

    # noise1 = const1 * (np.random.rand() - const2)
    # noise2 = const3 * (np.random.rand() - const4)

    x0 = ego['state']
    c = np.array(ego['ocpar'])
    t = dt * i
    eps = 10
    psc = 0.1

    phiRearEnd = ego['phiRearEnd']
    deltaSafetyDistance = ego['carlength']

    umax = 4
    umin = -6

    A = np.array([[1, 0], [-1, 0]])
    b = np.array([umax, -umin])

    vd = 0.5 * c[0] * t ** 2 + c[1] * t + c[2]
    u_ref = max(c[0] * t + c[1], 0)

    # CLF
    phi0 = -eps * (x0[1] - vd) ** 2
    phi1 = 2 * (x0[1] - vd)
    A = np.append(A, [[phi1, -1]], axis=0)
    b = np.append(b, [phi0])

    ip = ego['ip']
    for k in ip:
        index_ip = k - 1
        k_rear = ego['k_rear_end']
        s0 = que[k - 1]['state'][0]
        v0 = que[k - 1]['state'][1]

        LfB = v0 - x0[1] + k_rear * (s0 - x0[0] - phiRearEnd * x0[1] - deltaSafetyDistance)
        LgB = phiRearEnd

        A = np.append(A, [[LgB, 0]], axis=0)
        b = np.append(b, [LfB])

    index = ego['index']
    position = ego['position']
    for k in range(len(index)):
        for j in range(len(index[k])):
            if index[k][j] == -1:
                continue
            else:
                d1 = que[index[k][j]-1]['metric'][position[k][j] + 3] - que[index[k][j]-1]['state'][0] #6
                d2 = ego['metric'][k + 4] - x0[0] #4

            phiLateral = ego['phiLateral']
            k_lateral = ego['k_lateral'][k]
            L = ego['metric'][k + 4]
            bigPhi = phiLateral * x0[0] / L

            h = d2 - d1 - bigPhi * x0[1] - deltaSafetyDistance
            LgB = bigPhi
            LfB = que[index[k][j]-1]['state'][1] - x0[1] - phiLateral * x0[1]**2 / L + k_lateral*h
            if LgB != 0:
                A = np.append(A, [[LgB, 0]], axis=0)
                b = np.append(b, [LfB])

    vmin = 0.1

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
    H = matrix(b)

    try :
        options = {'show_progress': False}
        Solution = cvxopt.solvers.qp(P, q, G, H, options=options)
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
        u = matrix([umin/2, 0.0])  # [-cd * m * g, 0]

    a = (u[0], 0, 0)
    t = [0, 0.1]
    y0 = [x0[0], x0[1]]

    result = odeint(second_order_model, y0, t, args=a)
    rt = [result[-1,0], result[-1,1], a[0]]

    if rt[1] < 0:
        rt[1] = 0.01
        rt[0] = ego['state'][0] + 0.01
    if rt[0] < 0:
        time.sleep(1)

    return rt, infeasiblity

def OCBF_event(i, ego, que, flags):

    infeasiblity = 0
    vmax = 20
    dt = 0.1
    # noise1 = const1 * (np.random.rand() - const2)
    # noise2 = const3 * (np.random.rand() - const4)
    x0 = ego['state']
    c = np.array(ego['ocpar'])
    t = dt * i
    eps = 10
    psc = 0.1

    if ego['state'][0] > ego["metric"][-1]:
        v = 10
        u1 = 0
        x = ego['state'][0] + v * dt
        rt = [x, v, u1]
        return rt, 1

    elif 1 in flags:
        deltaSafetyDistance = ego["carlength"]
        # physical limitations on control inputs
        umax = 3
        umin = -6
        vmin = 0
        A = np.array([[1, 0], [-1, 0]])
        b = np.array([umax, -umin])

        # reference trajectory

        vd = 0.5 * c[0] * t ** 2 + c[1] * t + c[2]
        u_ref = max(c[0] * t + c[1], 0)

        # CLF
        phi0 = -eps * (x0[1] - vd) ** 2
        phi1 = 2 * (x0[1] - vd)
        A = np.append(A, [[phi1, -1]], axis=0)
        b = np.append(b, [phi0])

        ip = ego['ip']
        for k in range(len(ip)):
            # Extracting values
            k_rear = ego['k_rear_end']
            phiRearEnd = ego['phiRearEnd']

            index_ip = ip[k]-1

            v_tk, x_tk = x0[1], x0[0]
            vp_tk, xp_tk = que[index_ip]['state'][1], que[index_ip]['state'][0]

            C1_a = [phiRearEnd, +1, 0, -1]
            C1_b = -deltaSafetyDistance
            v_a = np.array([[1, 0, 0, 0], [-1, 0, 0, 0]])
            v_b = np.array([v_tk + s1, s1 - v_tk])
            x_a = np.array([[0, 1, 0, 0], [0, -1, 0, 0]])
            x_b = np.array([x_tk + s2, s2 - x_tk])
            vp_a = np.array([[0, 0, 1, 0], [0, 0, -1, 0]])
            vp_b = np.array([vp_tk + s1, s1 - vp_tk])
            xp_a = np.array([[0, 0, 0, 1], [0, 0, 0, -1]])
            xp_b = np.array([xp_tk + s2, s2 - xp_tk])

            A_lin = np.vstack((C1_a, v_a, x_a, vp_a, xp_a))
            b_lin = np.hstack((C1_b, v_b, x_b, vp_b, xp_b))
            f_lin = np.array([-k_rear * phiRearEnd - 1, -k_rear * 1, 1, k_rear * 1])

            options = {'disp': False}
            result = linprog(f_lin, A_ub=A_lin, b_ub=b_lin, options=options)

            if result.success:
                Lf_terms = result.fun - k_rear * deltaSafetyDistance
                Lg_term = phiRearEnd
            else:
                Lg_term = phiRearEnd
                Lf_terms = vp_tk - v_tk + k_rear * (xp_tk - x_tk - phiRearEnd * v_tk - deltaSafetyDistance)

            A = np.append(A, [[Lg_term, 0]], axis=0)
            b = np.append(b, [Lf_terms])
        #

        for k in range(len(ego['see'])):
            #if que[ego['see'][k]]['state'][0] >= ego['state'][0]:
            if que[ego['see'][k]]['lane'] == ego['lane']: #same lane
                k_rear = ego['k_rear_end']
                phiRearEnd = ego['phiRearEnd']
                v_tk = x0[1]
                x_tk = x0[0]
                xp_tk = que[ego['see'][k]]['state'][0]  # position of cav ip
                vp_tk = que[ego['see'][k]]['state'][1]  # velocity of cav ip
                C1_a = [phiRearEnd, +1, 0, -1]
                C1_b = -deltaSafetyDistance
                v_a = np.array([[1, 0, 0, 0], [-1, 0, 0, 0]])
                v_b = np.array([v_tk + s1, s1 - v_tk])
                x_a = np.array([[0, 1, 0, 0], [0, -1, 0, 0]])
                x_b = np.array([x_tk + s2, s2 - x_tk])
                vp_a = np.array([[0, 0, 1, 0], [0, 0, -1, 0]])
                vp_b = np.array([vp_tk + s1, s1 - vp_tk])
                xp_a = np.array([[0, 0, 0, 1], [0, 0, 0, -1]])
                xp_b = np.array([xp_tk + s2, s2 - xp_tk])

                A_lin = np.vstack((C1_a, v_a, x_a, vp_a, xp_a))
                b_lin = np.hstack((C1_b, v_b, x_b, vp_b, xp_b))
                f_lin = np.array([-k_rear * phiRearEnd - 1, -k_rear * 1, 1, k_rear * 1])
                options = {'disp': False}
                result = linprog(f_lin, A_ub=A_lin, b_ub=b_lin, options=options)

                if result.success:
                    Lf_terms = result.fun - k_rear * deltaSafetyDistance
                    Lg_term = phiRearEnd
                else:
                    Lg_term = phiRearEnd
                    Lf_terms = vp_tk - v_tk + k_rear * (xp_tk - x_tk - phiRearEnd * v_tk - deltaSafetyDistance)

                # Assuming A and b are defined before this code snippet
                A = np.append(A, [[Lg_term, 0]], axis=0)
                b = np.append(b, [Lf_terms])
        index = ego['index']
        position = ego['position']
        for k in range(len(index)):
            for j in range(len(index[k])):
                if index[k][j] == -1:
                    continue
                else:
                    v_tk = x0[1]
                    x_tk = x0[0]
                    index_ic = index[k][j] - 1
                    position_ic = position[k][j]
                    vl_tk = que[index_ic]['state'][1]
                    xl_tk = que[index_ic]['state'][0]
                    d1 = que[index_ic]['metric'][position_ic + 3] - que[index_ic]['state'][0]  # 6
                    d2 = ego['metric'][k + 4] - x0[0]  # 4

                k_lateral = ego['k_lateral'][k]
                phiLateral = ego['phiLateral']
                L = ego['metric'][k + 4]
                x_init = [v_tk, x_tk, vl_tk, xl_tk]

                def objective(x):
                    return x[2] - x[0] - phiLateral/L * x[0]**2 + k_lateral * ((ego['metric'][k + 4] - x[1]) -
                           (que[index_ic]['metric'][position_ic + 3] - x[3])- phiLateral/L * x[1] * x[0]
                                                                               + deltaSafetyDistance)

                # Constraint function
                def constraint(x):
                    return (-(ego['metric'][k + 4] - x[1]) + (que[index_ic]['metric'][position_ic + 3] - x[3]) +
                            phiLateral/L * x[1] * x[0] + deltaSafetyDistance)

                rt_slack, infeasiblity = OCBF_time(i, ego, que)

                # Optimization
                result = minimize(
                    fun=lambda x: objective(x),
                    x0=x_init,
                    constraints={'type': 'ineq', 'fun': lambda x: constraint(x)},
                    bounds=[(v_tk - s1, v_tk + s1), (x_tk - s2, x_tk + s2), (vl_tk - s1, vl_tk + s1), (xl_tk - s2, xl_tk + s2)]
                )

                fval_quad = result["fun"]

                if rt_slack[2] >= 0:
                    Lf_terms = fval_quad
                    Lg_term = phiLateral / L * max((x_tk - s2), 0.01)

                else:
                    Lf_terms = fval_quad
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
            rt[1] = 0.01
            rt[0] = ego['state'][0] + 0.01
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


def no_model_attacker(i, ego):
    deltax = 0.5*random.random()
    deltav =  deltax/0.1
    deltau = 0
    rt = [ego['state'][0] + deltax, deltav, ego['state'][2] + deltau]

    return rt

def random_init_attacker(i, ego):
    deltax = random.random()
    deltav = 0.05 * random.random()
    deltau = 0
    rt = [ego['state'][0] + deltax, deltav, ego['state'][2] + deltau]

    return rt


def no_rule_attacker(i, ego):
    vmax = 20
    dt = 0.1

    if ego['state'][0] > ego["metric"][-1]:
        v = 10
        u1 = 0
        x = ego['state'][0] + v * dt
        rt = [x, v, u1]
        return rt

    # noise1 = const1 * (np.random.rand() - const2)
    # noise2 = const3 * (np.random.rand() - const4)

    x0 = ego['state']
    c = np.array(ego['ocpar'])
    t = dt * i
    eps = 10
    psc = 0.1

    phiRearEnd = ego['phiRearEnd']
    deltaSafetyDistance = ego['carlength']

    umax = 4
    umin = -6

    A = np.array([[1, 0], [-1, 0]])
    b = np.array([umax, -umin])

    vd = 0.5 * c[0] * t ** 2 + c[1] * t + c[2]
    u_ref = max(c[0] * t + c[1], 0)

    # CLF
    phi0 = -eps * (x0[1] - vd) ** 2
    phi1 = 2 * (x0[1] - vd)
    A = np.append(A, [[phi1, -1]], axis=0)
    b = np.append(b, [phi0])

    ip = ego['ip']
    for k in ip:
        index_ip = k - 1
        k_rear = ego['k_rear_end']
        s0 = que[index_ip]['state'][0]
        v0 = que[index_ip]['state'][1]

        LfB = v0 - x0[1] + k_rear * (s0 - x0[0] - phiRearEnd * x0[1] - deltaSafetyDistance)
        LgB = phiRearEnd

        A = np.append(A, [[LgB, 0]], axis=0)
        b = np.append(b, [LfB])

    index = ego['index']
    position = ego['position']
    for k in range(len(index)):
        for j in range(len(index[k])):
            if index[k][j] == -1:
                continue
            else:
                d1 = que[index[k][j]-1]['metric'][position[k][j] + 3] - que[index[k][j]-1]['state'][0] #6
                d2 = ego['metric'][k + 4] - x0[0] #4

            phiLateral = ego['phiLateral']
            k_lateral = ego['k_lateral'][k]
            L = ego['metric'][k + 4]
            bigPhi = phiLateral * x0[0] / L

            h = d2 - d1 - bigPhi * x0[1] - deltaSafetyDistance
            LgB = bigPhi
            LfB = que[index[k][j]-1]['state'][1] - x0[1] - phiLateral * x0[1]**2 / L + k_lateral*h
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
    H = matrix(b)

    try :
        options = {'show_progress': False}
        Solution = cvxopt.solvers.qp(P, q, G, H, options=options)
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
        u = matrix([umin/2, 0.0])  # [-cd * m * g, 0]

    a = (u[0], 0, 0)
    t = [0, 0.1]
    y0 = [x0[0], x0[1]]

    result = odeint(second_order_model, y0, t, args=a)
    rt = [result[-1,0], result[-1,1], a[0]]

    if rt[1] < 0.1:
        rt[1] = 0

    if rt[0] < 0:
        time.sleep(1)

    return rt


def strategic_attacker(i, ego):
    vmax = 20
    dt = 0.1

    if ego['state'][0] > ego["metric"][-1]:
        v = 10
        u1 = 0
        x = ego['state'][0] + v * dt
        rt = [x, v, u1]
        return rt

    # noise1 = const1 * (np.random.rand() - const2)
    # noise2 = const3 * (np.random.rand() - const4)

    x0 = ego['state']
    c = np.array(ego['ocpar'])
    t = dt * i
    eps = 10
    psc = 0.1

    phiRearEnd = ego['phiRearEnd']
    deltaSafetyDistance = ego['carlength']

    umax = 4
    umin = -6

    A = np.array([[1, 0], [-1, 0]])
    b = np.array([umax, -umin])

    vd = 0.5 * c[0] * t ** 2 + c[1] * t + c[2]
    u_ref = max(c[0] * t + c[1], 0)

    # CLF
    phi0 = -eps * (x0[1] - vd) ** 2
    phi1 = 2 * (x0[1] - vd)
    A = np.append(A, [[phi1, -1]], axis=0)
    b = np.append(b, [phi0])

    ip = ego['ip']
    for k in ip:
        index_ip = k - 1
        k_rear = ego['k_rear_end']
        s0 = que[index_ip]['state'][0]
        v0 = que[index_ip]['state'][1]

        LfB = v0 - x0[1] + k_rear * (s0 - x0[0] - phiRearEnd * x0[1] - deltaSafetyDistance)
        LgB = phiRearEnd

        A = np.append(A, [[LgB, 0]], axis=0)
        b = np.append(b, [LfB])

    index = ego['index']
    position = ego['position']
    for k in range(len(index)):
        for j in range(len(index[k])):
            if index[k][j] == -1:
                continue
            else:
                d1 = que[index[k][j] - 1]['metric'][position[k][j] + 3] - que[index[k][j] - 1]['state'][0]  # 6
                d2 = ego['metric'][k + 4] - x0[0]  # 4

            phiLateral = ego['phiLateral']
            k_lateral = ego['k_lateral'][k]
            L = ego['metric'][k + 4]
            bigPhi = phiLateral * x0[0] / L

            h = d2 - d1 - bigPhi * x0[1] - deltaSafetyDistance
            LgB = bigPhi
            LfB = que[index[k][j] - 1]['state'][1] - x0[1] - phiLateral * x0[1] ** 2 / L + k_lateral * h
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
    H = matrix(b)

    try:
        options = {'show_progress': False}
        Solution = cvxopt.solvers.qp(P, q, G, H, options=options)
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
        u = matrix([umin / 2, 0.0])  # [-cd * m * g, 0]

    a = (u[0], 0, 0)
    t = [0, 0.1]
    y0 = [x0[0], x0[1]]

    result = odeint(second_order_model, y0, t, args=a)
    rt = [result[-1, 0], result[-1, 1], a[0]]

    if rt[1] < 0.1:
        rt[1] = 0

    if rt[0] < 0:
        time.sleep(1)

    return rt
