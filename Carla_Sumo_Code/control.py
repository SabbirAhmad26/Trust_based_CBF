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

def update_table(car, order):
    size = len([order])
    car['table'] = []
    print(order)
    for j in range(size):
        temp = np.zeros(15)
        # 1st: index, 2-29: CP 1-28, 30: current lane
        # 31: order No, that is the index of the vehicle in the queue
        temp[0] = car['que1'][int(order[j])]['id'][1]
        for i in range(4, len(car['que1'][int(order[j])]['id'])):
            temp[car['que1'][int(order[j])]['id'][i] + 1] = i - 3
        temp[13] = car['que1'][int(order[j])]['id'][2]
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
    s1 = 0.5
    s2 = 0.6
    s3 = 0.7

    noise_term_position = max(abs(const1 * const2), abs(const1 * (1 - const2)))
    noise_term_speed = max(abs(const3 * const4), abs(const3 * (1 - const4)))

    flag_i = 0
    flag_i1 = 0
    flag_ip = 0
    flag_ip_seen = 0
    flag_i_trust = 0

    if ego["state"][1] >= CAV_e["v_tk"][ego["id"][0]][1][0] + s1 - noise_term_speed or \
            ego["state"][1] <= CAV_e["v_tk"][ego["id"][0]][1][0] - s1 + noise_term_speed or \
            ego["state"][0] >= CAV_e["x_tk"][ego["id"][0]][1][0] + s2 - noise_term_position or \
            ego["state"][0] <= CAV_e["x_tk"][ego["id"][0]][1][0] - s2 + noise_term_position:
        flag_i = 1

    for k in range(len(ip)):
        if que1[k]["state"][1] >= CAV_e["v_tk"][ego["id"][0]][2][k] + s1 - noise_term_speed or \
                que1[k]["state"][1] <= CAV_e["v_tk"][ego["id"][0]][2][k] - s1 + noise_term_speed or \
                que1[k]["state"][0] >= CAV_e["x_tk"][ego["id"][0]][2][k] + s2 - noise_term_position or \
                que1[k]["state"][0] <= CAV_e["x_tk"][ego["id"][0]][2][k] - s2 + noise_term_position:
            flag_ip = 1

        if que1[k]["trust"][0] >= que1[k]["trust"][1] + s3 or que1[k]["trust"][0] <= que1[k]["trust"][1] - s3:
            flag_i_trust = 1

    for k in range(len(index)):
        for j in range(len(index[k])):
            if index[k][j] == -1:
                continue
            else:
                if que1[index[k][j]]["state"][1] >= CAV_e["v_tk"][ego["id"][0]][3 + k - 1][j] + s1 - noise_term_speed or \
                        que1[index[k][j]]["state"][1] <= CAV_e["v_tk"][ego["id"][0]][3 + k - 1][j] - s1 + noise_term_speed or \
                        que1[index[k][j]]["state"][0] >= CAV_e["x_tk"][ego["id"][0]][3 + k - 1][j] + s2 - noise_term_position or \
                        que1[index[k][j]]["state"][0] <= CAV_e["x_tk"][ego["id"][0]][3 + k - 1][j] - s2 + noise_term_position:
                    flag_i1 = 1

                if que1[index[k][j]]["trust"][0] >= que1[index[k][j]]["trust"][1] + s3 or \
                        que1[index[k][j]]["trust"][0] <= que1[index[k][j]]["trust"][1] - s3:
                    flag_i_trust = 1

    if ip_seen:
        if que1[ip_seen]["state"][1] >= CAV_e["v_tk"][ego["id"][0]][8][0] + s1 - noise_term_speed or \
                que1[ip_seen]["state"][1] <= CAV_e["v_tk"][ego["id"][0]][8][0] - s1 + noise_term_speed or \
                que1[ip_seen]["state"][0] >= CAV_e["x_tk"][ego["id"][0]][8][0] + s2 - noise_term_position or \
                que1[ip_seen]["state"][0] <= CAV_e["x_tk"][ego["id"][0]][8][0] - s2 + noise_term_position:
            flag_ip_seen = 1

    return flag_i,flag_i1,flag_ip,flag_ip_seen,flag_i_trust

def OCBF_time(i, one, que, ip, index, position, ilc, geometry_road):
    L = geometry_road['L']
    vmax = 20

    if one['state'][0] > L:
        v = vmax
        u1 = 0
        x = one['state'][0] + vMax * dt
        v = vmax
        u1 = 0
        x = one['state'][0] + vmax * dt
        rt = [x, v, u1]
        return rt

    noise1 = const1 * (np.random.rand() - const2)
    noise2 = const3 * (np.random.rand() - const4)

    x0 = one['state']
    c = np.array(one['ocpar'])
    t = dt * i
    eps = 10
    psc = 0.1

    phiRearEnd = one['phiRearEnd']
    phiLateral = one['phiLateral']

    deltaSafetyDistance = 0
    l = 7 - 3.5 * np.sqrt(3)

    umax = 3
    umin = -3
    A = matrix([[1, 0], [-1, 0]])
    b = matrix([umax, -umin])

    if one['id'][1] != 1:
        vd = max(0.5 * c[0] * t**2 + c[1] * t + c[2], 12)
        u_ref = min(c[0] * t + c[1], 0)
    else:
        vd = 6
        u_ref = 0

    # CLF
    phi0 = -eps * (x0[1] - vd) ** 2
    phi1 = 2 * (x0[1] - vd)
    A = sparse([A, matrix([phi1,-1],(1,2))])
    b = sparse([b, phi0])

    if ip != -1:
        k_rear = one['k_rear']
        s0 = que[ip]['state'][0]
        v0 = que[ip]['state'][1]

        LfB = v0 - x0[1] + k_rear * (s0 - x0[0] - phiRearEnd * x0[1] - deltaSafetyDistance)
        LgB = phiRearEnd
        A=sparse([A,matrix([LgB,0],(1,2))])
        b = matrix([b, LfB])

    for k in range(len(index)):
        if index[k] == -1:
            continue
        else:
            d1 = que[index[k]]['metric'][position[k] + 4] - que[index[k]]['state'][0]
            d2 = one['metric'][k + 4] - x0[0]

        k_lateral = one['k_lateral'][k]
        L = one['metric'][k + 4]
        bigPhi = phiLateral * x0[0] / L

        v0 = que[index[k]]['state'][1]
        h = d2 - d1 - bigPhi * x0[1]
        LgB = bigPhi
        LfB = v0 - x0[1] - k_lateral * (phiLateral * x0[1]**2 / L - deltaSafetyDistance)

        if LgB != 0:
            A = sparse([A,matrix([LgB,0],(1,2))])
            b = matrix([b, LfB + hf])

    vmin = 0
    A_vmax = matrix([1.0, 0.0],(1,2))  # CBF for max and min speed
    b_vmax = vmax - x0[1]
    A_vmin = matrix([-1.0, 0.0],(1,2))
    b_vmin = x0[1] - vmin
    A = matrix([A, A_vmax, A_vmin])
    b = matrix([b, b_vmax, b_vmin])

    H = matrix([[1.0, 0.0], [0.0, psc]])
    F = matrix([-u_ref, 0.0])
    options = {'show_progress': False}
    sol = cvxopt.solvers.qp(matrix(H), 
                            matrix(F), 
                            matrix(A), 
                            matrix(b), 
                            options=options)

    if sol['status'] == 'optimal':
        u = sol['x']
        a = u[0]
    else:
        # Handle the case when no optimal solution is found
        u = matrix([0.0, 0.0])  # [-cd * m * g, 0]

    a = (u[0],0,0) 
    t = [0, 0.1]    
    y0 = [x0[0],x0[1]]

    result = odeint(second_order_model, y0, t, args=a)
    rt = [result[-1, 0], result[-1, 1], a[0]]
    return rt

def OCBF_event(i, one, que, ip, index, position, ilc, CAV_e, geometry_road):

    L = geometry_road.L
    vmax = 20
    infeasiblity = 0
    
    if one['state'][0] > L:
            v = vmax
            u1 = 0
            x = one['state'][0] + vmax * 0.1
            rt = [x, v, u1]
    
    noise1 = const1 * (np.random.rand() - const2)
    noise2 = const3 * (np.random.rand() - const4)
    x0 = one.state
    c = np.double(one.ocpar)
    t = dt * i
    eps = 10
    psc = 0.1
    deltaSafetyDistance = 0
    l = 7-3.5*math.sqrt(3); # Suppose that the vehicle change lanes along with the line whose slope is 30 degree
    
    # physical limitations on control inputs
    umax = 3
    umin = -3
    A = matrix([[1, 0], [-1, 0]])
    A = A.trans()
    b = matrix([umax, -umin])
    
    # reference trajectory
    vd = 0.5 * c[0] * t ** 2 + c[1] * t + c[2]
    u_ref = c[0] * t + c[1]
    
    # CLF
    phi0 = -eps * (x0[1] - vd) ** 2
    phi1 = 2 * (x0[1] - vd)
    A = sparse([A, matrix([phi1,-1],(1,2))])
    b = sparse([b, phi0])
    
    for k in range(len(ip)):
        if one['id'][1] == 2:
            continue
    
        # Extracting values
        k_rear = one['k_rear']
        phiRearEnd = one['phiRearEnd']
        
        v_tk, x_tk = x0[1], x0[0]
        vp_tk, xp_tk = que[ip[k]]['state'][1], que[ip[k]]['state'][0]
        
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
        
        A=sparse([A,matrix([Lg_term, 0],(1,2))])
        b = matrix([b, Lf_terms])
    
    for k in range(len(one['see'])):
        # if numel(intersect(one.see(k), ip))
        if que[one['see'][k]]['id'][2] == one['id'][2] and que[one['see'][k]]['state'][0] >= one['state'][0]:
            k_rear = one['k_rear']
            phiRearEnd = one['phiRearEnd']
            v_tk = x0[1]
            x_tk = x0[0]
            xp_tk = que[one['see'][k]]['state'][0]  # position of cav ip
            vp_tk = que[one['see'][k]]['state'][1]  # velocity of cav ip
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
            A=sparse([A,matrix([Lg_term, 0],(1,2))])
            b = matrix([b, Lf_terms])
    
    for k in range(len(index)):
        for j in range(len(index[k])):
            if index[k][j] == -1:
                continue
            else:
                v_tk = x0[1]
                x_tk = x0[0]
                vl_tk = que[index[k][j]]['state'][1]
                xl_tk = que[index[k][j]]['state'][0]
                d1 = que[index[k][j]]['metric'][position[k][j] + 4] - xl_tk
                d2 = one['metric'][k + 4] - x_tk
    
            k_lateral = one['k_lateral'][k]
            phiLateral = one['phiLateral']
    
            bigPhi = phiLateral * x0[0] / L
            x_init = [v_tk, x_tk, vl_tk, xl_tk]
            lb = [v_tk - s1, x_tk - s2, vl_tk - s1, xl_tk - s2]
            ub = [v_tk + s1, x_tk + s2, vl_tk + s1, xl_tk + s2]
            
            def fun(x):
                return x[2] - x[0] - phiLateral / L * x[0]**2 + k_lateral * (
                    (one['metric'][k + 4] - x[1]) - (que[index_1]['metric'][position_1 + 4] - x[3])
                    - phiLateral / L * x[2] * x[0] - deltaSafetyDistance)
    
            def constraint(x):
                return -(one['metric'][k + 4] - x[1]) + (que[index_1]['metric'][position_1 + 4] - x[3]) + \
                       phiLateral / L * x[2] * x[0] + deltaSafetyDistance
            
            def nonlinfcn(x):
                return constraint(x), 0
            
            Aeq = []
            beq = []
            A_quad = []
            b_quad = []
            lb = [v_tk - s1, x_tk - s2, vl_tk - s1, xl_tk - s2]
            ub = [v_tk + s1, x_tk + s2, vl_tk + s1, xl_tk + s2]
            
            index_1 = index[k][j]
            position_1 = position[k][j]
            rt_slack = OCBF_time(i, one, que, ip, index_1, position_1, ilc, geometry_road)
            
            result = fmin_slsqp(fun, x_init, bounds=list(zip(lb, ub)), f_eqcons=nonlinfcn)
            fval_quad = result[2]  # Assuming the third element of the result corresponds to the cost value
            
            if rt_slack[2] >= 0:
                Lf_terms = fval_quad  # Assuming the third element of the result corresponds to the cost value
                Lg_term = phiLateral / L * (x_tk - s2)
                A=sparse([A,matrix([Lg_term, 0],(1,2))])
                b = matrix([b, Lf_terms])
            else:
                Lf_terms = fval_quad # Assuming the third element of the result corresponds to the cost value
                Lg_term = phiLateral / L * (x_tk + s2)
                A=sparse([A,matrix([Lg_term, 0],(1,2))])
                b = matrix([b, Lf_terms])
            
    vmax = 15
    vmin = 0
    A_vmax = matrix([1.0, 0.0],(1,2))  # CBF for max and min speed
    b_vmax = vmax - x0[1]
    A_vmin = matrix([-1.0, 0.0],(1,2))
    b_vmin = x0[1] - vmin
    A = matrix([A, A_vmax, A_vmin])
    b = matrix([b, b_vmax, b_vmin])
    
    H = matrix([[1.0, 0.0], [0.0, psc]])
    F = matrix([-u_ref, 0.0])
    
    options = {'show_progress': False}
    sol = cvxopt.solvers.qp(matrix(H), 
                            matrix(F), 
                            matrix(A), 
                            matrix(b), 
                            options=options)
    
    if sol['status'] == 'optimal':
        u = sol['x']
        a = u[0]
    else:
        # Handle the case when no optimal solution is found
        u = matrix([-3.0, 0.0])  # [-cd * m * g, 0]
        cnt += 1
        infeasibility = 1
    
    a = (u[0],0,0) 
    t = [0, 0.1]
    y0 = [x0[0],x0[1]]
    
    result = odeint(second_order_model, y0, t, args=a)
    rt = [result[-1, 0], result[-1, 1], a[0]]
    
    if rt[1] < 0:
        rt[1] = 0
    
    if rt[0] < 0:
        time.sleep(1)

    return rt, CAV_e, infeasiblity

