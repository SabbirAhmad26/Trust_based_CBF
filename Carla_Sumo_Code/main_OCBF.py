import numpy as np
import time
import numpy as np
from scipy.optimize import linprog
from cvxopt import matrix, solvers, sqrt
import cvxopt
import cvxopt.solvers
from scipy.integrate import odeint
from cvxopt import matrix, sparse


def tic():
    return time.time()


# Function to mimic toc behavior
def toc(start_time):
    elapsed_time = time.time() - start_time
    # print("Elapsed time: {:.5f} seconds".format(elapsed_time))


def second_order_model(x, t, a, noise1, noise2):
    m = 1
    f0 = 0.1
    f1 = 5
    f2 = 0.25
    dx = [0, 0]
    pos, vel = x
    dx[0] = vel + noise1
    dx[1] = (1 / m) * a + noise2
    return dx


def search_for_conflictCAVS_new(table, egocar):
    index = []
    position = []

    # Find k
    k = None
    for i in range(len(table)):
        if table[i][0] == egocar['id'][1]:
            k = i
            break

    ip = -1
    # Find ip
    for j in range(k - 1, 0, -1):
        if table[j][29] == table[k][29]:
            ip = table[j][30]
            break

    # Iterate over egocar['id']
    for i in range(4, len(egocar['id'])):
        flag = 0
        # Search for values in table
        for j in range(k - 1, 0, -1):
            if table[j][egocar['id'][i]] > 0:
                index.append(table[j][30])
                position.append(table[j][egocar['id'][i]])
                flag = 1
                break

        if flag == 0:
            index.append(-1)
            position.append(-1)
    return [ip, index, position]


def search_i_lanechange(que, new):
    que_len = len(que)
    index = []

    if new['id'][2] == 1:
        lane = 2
        action = 2
    elif new['id'][2] == 2:
        lane = 1
        action = 3
    elif new['id'][2] == 3:
        lane = 4
        action = 2
    elif new['id'][2] == 4:
        lane = 3
        action = 3
    elif new['id'][2] == 5:
        lane = 6
        action = 2
    elif new['id'][2] == 6:
        lane = 5
        action = 3
    elif new['id'][2] == 7:
        lane = 8
        action = 2
    elif new['id'][2] == 8:
        lane = 7
        action = 3

    for i in range(que_len, 0, -1):
        if que[i - 1]['id'][1] == new['id'][1]:
            k = i
            break

    for i in range(k - 1, 0, -1):
        if que[i - 1]['id'][2] == lane and que[i - 1]['id'][0] == action:
            index = i
            break
    return index


def OCBF_SecondOrderDynamics(i, one, que, ip, index, position, ilc):
    L = 300
    vMax = 15

    if one['state'][0] > L:
        v = vMax
        u1 = 0
        x = one['state'][0] + vMax * 0.1
        rt = [x, v, u1]
        # return is not allowed here in Python, you may need to handle it differently

    global u, cnt, noise1, noise2
    # noise1 = 4 * (rand() - 0.5)
    # noise2 = 0.1 * (rand() - 0.5)
    noise1 = 0
    noise2 = 0
    x0 = matrix(one['state'])
    c = matrix(one['ocpar'].astype(float))
    t = 0.1 * i
    eps = 10
    psc = 0.1

    phiRearEnd = 0
    phiLateral = 1.8
    deltaSafetyDistance = 10
    l = 7 - 3.5 * sqrt(3)

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
    A = sparse([A, matrix([phi1, -1], (1, 2))])
    b = sparse([b, phi0])

    # rear-end safety constraints
    if ip != -1:
        s0 = que[ip]['state'][0]
        h = s0 - x0[0] - phiRearEnd * x0[1] - deltaSafetyDistance
        v0 = que[ip]['state'][1]
        uminValue = abs(umin)
        hf = h - 0.5 * (v0 - x0[1]) ** 2 / uminValue

        if x0[1] <= v0 or hf < 0:
            p = 1
            LgB = 1
            LfB = 2 * p * (v0 - x0[1]) + p ** 2 * h
            A = sparse([A, matrix([LgB, 0], (1, 2))])
            b = matrix([b, LfB])
        else:
            LgB = phiRearEnd - (v0 - x0[1]) / uminValue
            LfB = v0 - x0[1]
            if LgB != 0:
                A = sparse([A, matrix([LgB, 0], (1, 2))])
                b = matrix([b, LfB + hf])

    # lane change safety constraint
    if ilc:
        d1 = que[ilc]['metric'][4] - que[ilc]['state'][0]
        d2 = que[ilc]['metric'][4] - x0[0] - l
        L = que[ilc]['metric'][4]
        v0 = que[ilc]['state'][1]
        bigPhi = phiLateral * x0[0] / L
        h = d2 - d1 - bigPhi * x0[1]
        uminValue = abs(umin)
        hf = d2 - d1 - 0.5 * (v0 - x0[1]) ** 2 / uminValue - phiLateral * v0 * (
                x0[0] + 0.5 * (x0[1] ** 2 - v0 ** 2) / uminValue) / L
        if x0[1] <= v0 or hf < 0:
            LgB = bigPhi
            LfB = v0 - x0[1] - phiLateral * x0[1] ** 2 / L
            if LgB != 0:
                A = sparse([A, matrix([LgB, 0], (1, 2))])
                b = matrix([b, LfB + h])
        else:
            LgB = phiLateral * v0 * x0[1] / uminValue / L - (v0 - x0[1]) / uminValue
            LfB = v0 - x0[1] - phiLateral * v0 * x0[1] / L
            if LgB != 0:
                A = sparse([A, matrix([LgB, 0], (1, 2))])
                b = matrix([b, LfB + hf])

    # lateral safety constraints
    for k in range(len(index)):
        if index[k] == -1:
            continue
        else:
            d1 = que[index[k]]['metric'][position[k] + 4] - que[index[k]]['state'][0]
            d2 = one['metric'][k + 4] - x0[0]

        L = one['metric'][k + 4]
        v0 = que[index[k]]['state'][1]
        bigPhi = phiLateral * x0[0] / L
        h = d2 - d1 - bigPhi * x0[1]
        uminValue = abs(umin)
        hf = d2 - d1 - 0.5 * (v0 - x0[1]) ** 2 / uminValue - phiLateral * v0 * (
                x0[0] + 0.5 * (x0[1] ** 2 - v0 ** 2) / uminValue) / L
        if x0[1] <= v0 or hf < 0:
            LgB = bigPhi
            LfB = v0 - x0[1] - phiLateral * x0[1] ** 2 / L
            if LgB != 0:
                A = sparse([A, matrix([LgB, 0], (1, 2))])
                b = matrix([b, LfB + h])
        else:
            LgB = phiLateral * v0 * x0[1] / uminValue / L - (v0 - x0[1]) / uminValue
            LfB = v0 - x0[1] - phiLateral * v0 * x0[1] / L
            if LgB != 0:
                A = sparse([A, matrix([LgB, 0], (1, 2))])
                b = matrix([b, LfB + hf])

    vmax = 15
    vmin = 0
    A_vmax = matrix([1.0, 0.0], (1, 2))  # CBF for max and min speed
    b_vmax = vmax - x0[1]
    A_vmin = matrix([-1.0, 0.0], (1, 2))
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

    a = (u[0], 0, 0)
    t = [0, 0.1]
    y0 = [x0[0], x0[1]]

    result = odeint(second_order_model, y0, t, args=a)
    rt = [result[-1, 0], result[-1, 1], a[0]]

    if rt[1] < 0:
        rt[1] = 0

    if rt[0] < 0:
        time.sleep(1)

    return rt


def oc(i, one, que, ip):
    violation = one['violation']
    x0 = one['state'][0]
    v0 = one['state'][1]
    t = 0.1 * i
    vMax = 15
    uMax = 3
    deltaSafetyDistance = 10
    L = 300

    parameter = np.array(one['ocpar'])

    if x0 > L or (violation != 4 and t > parameter[5]):
        v = vMax
        u = 0
        x = x0 + vMax * 0.1
        rt = [x, v, u]
        # Assuming return statement means exit the function
        # If not, remove the return statement and adjust accordingly
        return

    if violation == 4 and t > parameter[11]:
        v = vMax
        u = 0
        x = x0 + vMax * 0.1
        rt = [x, v, u]
        return

    if violation == 0 or (violation == 1 and t <= parameter[4]) or (violation == 2 and t <= parameter[4]):
        ak = parameter[0]
        bk = parameter[1]
        ck = parameter[2]
        dk = parameter[3]
    elif violation == 1:
        u = 0
        v = vMax
        x = x0 + vMax * 0.1
        rt = [x, v, u]
        return
    elif violation == 2:
        u = uMax
        v = v0 + u * 0.1
        x = x0 + v0 * 0.1 + 0.5 * u * 0.1 ** 2
        rt = [x, v, u]
        return
    elif violation == 3:
        if t < parameter[4]:
            ak = parameter[0]
            bk = parameter[1]
            ck = parameter[2]
            dk = parameter[3]
        else:
            u = que[ip]['state'][2]
            v = que[ip]['state'][1]
            x = que[ip]['state'][0] - deltaSafetyDistance
            rt = [x, v, u]
            return
    elif violation == 4:
        if t <= parameter[4]:
            ak = parameter[0]
            bk = parameter[1]
            ck = parameter[2]
            dk = parameter[3]
        elif t > parameter[4] and t < parameter[10]:
            u = que[ip]['state'][2]
            v = que[ip]['state'][1]
            x = que[ip]['state'][0] - deltaSafetyDistance
            rt = [x, v, u]
            return
        elif t <= parameter[11]:
            ak = parameter[6]
            bk = parameter[7]
            ck = parameter[8]
            dk = parameter[9]

    v = 0.5 * ak * t ** 2 + bk * t + ck
    u = ak * t + bk
    x = 1 / 6 * ak * t ** 3 + 0.5 * bk * t ** 2 + ck * t + dk

    rt = [x, v, u]
    return rt


def fuel_consumption(init, u, v):
    b = [0.1569, 0.02450, -0.0007415, 0.00005975]
    c = [0.07224, 0.09681, 0.001075]

    if u > 0:
        result = init + 0.1 * (
                    u * (c[0] + c[1] * v + c[2] * v ** 2) + (b[0] + b[1] * v + b[2] * v ** 2 + b[3] * v ** 3))
    else:
        result = init + 0.1 * (b[0] + b[1] * v + b[2] * v ** 2 + b[3] * v ** 3)

    return result


def main_OCBF(i, car, mod):
    order = np.zeros(len(car['table']))

    # Extract the 31st element from each string in car.table and store in order
    for j in range(len(car['table'])):
        order[j] = car['table'][j][30]

        # Get the length of the resulting order array
    len_order = len(order)
    for k in range(len_order):
        ip, index, position = search_for_conflictCAVS_new(car['table'], car['que1'][int(order[k])])

        # for lane change
        # ilc = search_i_lanechange(car.que1, car.que1{k});

        # if lane change is disallowed
        ilc = []
        # Example usage
        start_time = tic()

        # Your code to measure the time for
        if mod == 1:
            rt = OCBF_SecondOrderDynamics(i, car['que1'][int(order[k])], car['que1'], ip, index, position, ilc)
            # for complicated vehicle dynamics
            # rt = OCBF_ComplicatedVehicleDynamicsNoise(i, car['que1'][order[k]], car['que1'], ip, index, position, ilc)
        elif mod == 2:
            rt = oc(i, car['que1'][int(order[k])], car['que1'], ip)
            # rt = OCBF2(i, car['que1'][k], car['que1'], ip)
        record = toc(start_time)

        order_k = order[k]  # Assuming 'order' is a list or array in Python

        car['que1'][int(order[k])]['state'] = rt

        if rt[0] <= car['que1'][int(order[k])]['metric'][-1]:
            car['que1'][int(order[k])]['metric'][0] += 0.1
            car['que1'][int(order[k])]['metric'][1] = fuel_consumption(car['que1'][int(order[k])]['metric'][1], rt[2],
                                                                       rt[1])
            car['que1'][int(order[k])]['metric'][2] += 0.1 * 0.5 * rt[2] ** 2
    return car

# car = {'cars': 1, 'cars1': 2, 'cars2': 0, 'Car_leaves': 0, 'Car_leavesMain': 0, 'Car_leavesMerg': 0, 'que1': [{'state': np.array([0., 10., 0.]), 'lane': 2, 'id': [3, 1.0, 2, 8, 28], 'metric': [0, 0, 0, 622.0, 309.03207887907064], 'deltaTurn': 2, 'ocpar': np.array([-0.03770785,  1.15388122,  8.8649727 , -9.43562867, 30.60055723,
#       26.5196769 ])}], 'que2': [], 'table': [np.array([1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
#       0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 2., 0.])]}
# i = 10
# one = car['que1'][0]
# que = car['que1']
# ip = -1
# index =[-1]
# position = [-1]
# ilc = []
# main_OCBF(i, car, mod=1)

# que1= [{'state': [1.0055808669287933, 10.111617336912635, 1.116173369126385], 'lane': 2, 'id': [3, 1.0, 2, 8, 28], 'metric': [0.1, 0.1686536214441233, 0.06229214949734727, 622.0, 309.03207887907064], 'deltaTurn': 2, 'ocpar': np.array([-0.03770785,  1.15388122,  8.8649727 , -9.43562867, 30.60055723,
#           26.5196769 ])}]

# check_leave(que1)
