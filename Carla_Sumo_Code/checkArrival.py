import math

import numpy as np

# import tmin
from OCT1 import OCT1
from get_L_1 import get_L_1
from search_i_p import search_i_p
from findMP import find_MPs


# class Test:
# def __init__(self):
# self.sum = 0
# def add(self):
# self.sum += 1


def check_arrival(i, init_queue, car, pen, pointer, trajs):
    l = 7 - 3.5 * math.sqrt(3)  # Suppose that the vehicle changes lanes along with the line whose slope is 30 degrees
    L = 300  # The length of the control zone, which is L1 in the paper
    w = 3.5  # Lane width
    r = 4.0  # The radius of the small circle for a right-turn

    alpha1 = math.degrees(math.asin((r + 0.5 * w) / (r + 2.5 * w)))
    alpha2 = math.degrees(math.asin((r + 1.5 * w) / (r + 2.5 * w)))

    ratio1 = alpha1 / 360
    ratio2 = alpha2 / 360
    ratio3 = 0.25 - ratio2
    ratio4 = 0.25 - ratio1

    lowerLaneChanging = 57 + l
    upperLaneChanging = 250 + l



    # new_state = [0, init_queue[4], 0]

    L_end = L * 2 + 4 * w + 2 * r

    # Assuming the 6th is he action
    # action 1: go straight, 2: turn left, 3: turn right
    # lane 1

    trajs = trajs.splitlines()
    trajs = list(filter(None, trajs))

    righta = [i for i, line in enumerate(trajs) if line == 'Right A:']
    lefta = [i for i, line in enumerate(trajs) if line == 'Left A:']
    straighta = [i for i, line in enumerate(trajs) if line == 'straight A:']

    originb = [i for i, line in enumerate(trajs) if line == 'Origin Lane B:']
    rightb = [i for i, line in enumerate(trajs) if line == 'Right B:']
    leftb = [i for i, line in enumerate(trajs) if line == 'Left B:']
    straightb = [i for i, line in enumerate(trajs) if line == 'Straight B:' or line == 'Straight B: ']

    originc = [i for i, line in enumerate(trajs) if line == 'Origin Lane C:']
    rightc = [i for i, line in enumerate(trajs) if line == 'Right C:']
    leftc = [i for i, line in enumerate(trajs) if line == 'Left C:']
    straightc = [i for i, line in enumerate(trajs) if line == 'Straight C:']

    origind = [i for i, line in enumerate(trajs) if line == 'Origin Lane D:' or line == 'Origin Lane D: ']
    rightd = [i for i, line in enumerate(trajs) if line == 'Right D:' or line == 'Right D: ']
    leftd = [i for i, line in enumerate(trajs) if line == 'Left D:']
    straightd = [i for i, line in enumerate(trajs) if line == 'Straight D']

    class new:
        def __init__(self, lane, id, metric, j, str, decision):
            self.state = np.array([0, init_queue[6], 0], dtype=float)
            self.lane = lane
            self.id = id
            self.metric = metric
            self.j = j
            self.str = str
            self.decision = decision

    # if init_queue[9] == 1 and init_queue[10] == 1:
    #     new_lane = 1  # original_lane
    #     new_id = [1, pen, 1, 1, 18, 19, 20, 21,
    #               22]  # action, vehicle_id, current_lane, destination_lane, MP1, MP2.....MPn
    #     new_metric = [0, 0, 0]  # time, fuel, energy, distance, MP1_dis, MP2_dis
    #     new_j = 31
    #     str = trajs[leftb[0] + 1:straightb[0]]
    #     New = new(new_lane, new_id, new_metric, new_j, str)
    if init_queue[9] == 1 and init_queue[10] == 2:
        new_lane = 1  # original_lane
        new_decision = 2
        new_id = [2, pen, 1, 3, 6, 4]  # action, vehicle_id, current_lane, destination_lane, MP1, MP2.....MPn
        new_metric = [0, 0, 0]  # time, fuel, energy, distance, MP1_dis, MP2_dis
        new_j = 31
        str = trajs[leftb[0] + 1:straightb[0]]
        New = new(new_lane, new_id, new_metric, new_j, str, new_decision)
    # elif init_queue[9] == 1 and init_queue[10] == 3:
    #     new_lane = 1
    #     new_id = [3, pen, 1, 8, 1, 28]  # action, vehicle_id, origin_lane, cur_lane, MP1, MP2.....MPn
    #     new_metric = [0, 0, 0]  # time, fuel, energy, distance, MP1_dis, MP2_dis
    #     new_j = 31
    #     str = trajs[leftb[0] + 1:straightb[0]]
    #     New = new(new_lane, new_id, new_metric, new_j, str)

    # lane 2

    elif init_queue[9] == 2 and init_queue[10] == 1:
        new_lane = 2
        new_decision = 1
        new_id = [1, pen, 2, 2, 8, 9, 10, 11]  # queue_id, vehicle_id, origin_lane, cur_lane, MP1, MP2.....MPn
        new_metric = [0, 0, 0]  # time, fuel, energy, distance, MP1_dis, MP2_dis
        new_j = 31
        str = trajs[straightb[0] + 1:originc[0]]
        New = new(new_lane, new_id, new_metric, new_j, str, new_decision)
    # elif init_queue[9] == 2 and init_queue[10] == 2:
    #     new_lane = 2
    #     new_id = [2, pen, 2, 3, 1, 18, 16, 13, 9]  # queue_id, vehicle_id, origin_lane, cur_lane, MP1, MP2.....MPn
    #     new_metric = [0, 0, 0]  # time, fuel, energy, distance, MP1_dis, MP2_dis
    #     new_j = 31
    #     str = trajs[leftc[0] + 1:rightc[0]]
    #     New = new(new_lane, new_id, new_metric, new_j, str)

    elif init_queue[9] == 2 and init_queue[10] == 3:
        new_lane = 2
        new_decision = 3
        new_id = [3, pen, 2, 8, 12]  # queue_id, vehicle_id, origin_lane, cur_lane, MP1, MP2.....MPn
        new_metric = [0, 0, 0]  # time, fuel, energy, distance, MP1_dis, MP2_dis
        new_j = 31
        str = trajs[rightb[0] + 1:leftb[0]]
        New = new(new_lane, new_id, new_metric, new_j, str, new_decision)

    # lane 3

    # elif init_queue[9] == 3 and init_queue[10] == 1:
    #     new_lane = 3
    #     # new_state = [100, init_queue[3], 0]  # Assuming the commented state assignment is required
    #     new_id = [1, pen, 3, 3, 26, 21, 17, 14, 9]  # queue_id, vehicle_id, origin_lane, cur_lane, MP1, MP2.....MPn
    #     new_metric = [0, 0, 0]  # time, fuel, energy, distance, MP1_dis, MP2_dis
    #     new_j = 31
    #     str = trajs[leftb[0] + 1:straightb[0]]
    #     New = new(new_lane, new_id, new_metric, new_j, str)

    elif init_queue[9] == 3 and init_queue[10] == 2:
        new_lane = 3
        new_decision = 2
        # new_state = [100, init_queue[3], 0]  # Assuming the commented state assignment is required
        new_id = [2, pen, 3, 5, 9, 6]  # queue_id, vehicle_id, origin_lane, cur_lane, MP1, MP2.....MPn
        new_metric = [0, 0, 0]  # time, fuel, energy, distance, MP1_dis, MP2_dis
        new_j = 31
        str = trajs[leftc[0] + 1:rightc[0]]
        New = new(new_lane, new_id, new_metric, new_j, str, new_decision)

    # elif init_queue[9] == 3 and init_queue[10] == 3:
    #     new_lane = 3
    #     # new_state = [100, init_queue[3], 0]  # Assuming the commented state assignment is required
    #     new_id = [3, pen, 3, 2, 2, 27]  # queue_id, vehicle_id, origin_lane, cur_lane, MP1, MP2.....MPn
    #     new_metric = [0, 0, 0]  # time, fuel, energy, distance, MP1_dis, MP2_dis
    #     new_j = 31
    #     str = trajs[leftb[0] + 1:straightb[0]]
    #     New = new(new_lane, new_id, new_metric, new_j, str)

    # lane 4

    elif init_queue[9] == 4 and init_queue[10] == 1:
        new_lane = 4
        new_decision = 1
        # new_state = [100, init_queue[3], 0]  # Assuming the commented state assignment is required
        new_id = [1, pen, 4, 4, 10, 7, 5, 1]  # queue_id, vehicle_id, origin_lane, cur_lane, MP1, MP2.....MPn
        new_metric = [0, 0, 0]  # time, fuel, energy, distance, MP1_dis, MP2_dis
        new_j = 31
        str = trajs[straightc[0] + 1:origind[0]]
        New = new(new_lane, new_id, new_metric, new_j, str, new_decision)

    # elif init_queue[9] == 4 and init_queue[10] == 2:
    #     new_lane = 4
    #     # new_state = [100, init_queue[3], 0]  # Assuming the commented state assignment is required
    #     new_id = [2, pen, 4, 5, 2, 26, 20, 16, 11]  # queue_id, vehicle_id, origin_lane, cur_lane, MP1, MP2.....MPn
    #     new_metric = [0, 0, 0]  # time, fuel, energy, distance, MP1_dis, MP2_dis
    #     new_j = 31
    #     str = trajs[leftb[0] + 1:straightb[0]]
    #     New = new(new_lane, new_id, new_metric, new_j, str)

    elif init_queue[9] == 4 and init_queue[10] == 3:
        new_lane = 4
        new_decision = 3
        # new_state = [100, init_queue[3], 0]  # Assuming the commented state assignment is required
        new_id = [3, pen, 4, 2, 11]  # queue_id, vehicle_id, origin_lane, cur_lane, MP1, MP2.....MPn
        new_metric = [0, 0, 0, ]  # time, fuel, energy, distance, MP1_dis, MP2_dis
        new_j = 31
        str = trajs[rightc[0] + 1:straightc[0]]
        New = new(new_lane, new_id, new_metric, new_j, str, new_decision)

    # lane 5

    # elif init_queue[9] == 5 and init_queue[10] == 1:
    #     new_lane = 5
    #     # new_state = [200, init_queue[3], 0]  # Assuming the commented state assignment is required
    #     new_id = [1, pen, 5, 5, 15, 14, 13, 12, 11]  # queue_id, vehicle_id, origin_lane, cur_lane, MP1, MP2.....MPn
    #     new_metric = [0, 0, 0]  # time, fuel, energy, distance, MP1_dis, MP2_dis
    #     new_j = 31
    #     str = trajs[leftb[0] + 1:straightb[0]]
    #     New = new(new_lane, new_id, new_metric, new_j, str)

    elif init_queue[9] == 5 and init_queue[10] == 2:
        new_lane = 5
        new_decision = 2
        # new_state = [200, init_queue[3], 0]  # Assuming the commented state assignment is required
        new_id = [2, pen, 5, 7, 7, 9]  # queue_id, vehicle_id, origin_lane, cur_lane, MP1, MP2.....MPn
        new_metric = [0, 0, 0]  # time, fuel, energy, distance, MP1_dis, MP2_dis
        new_j = 1
        str = trajs[leftd[0] + 1:straightd[0]]
        New = new(new_lane, new_id, new_metric, new_j, str, new_decision)

    # elif init_queue[9] == 5 and init_queue[10] == 3:
    #     new_lane = 5
    #     # new_state = [200, init_queue[3], 0]  # Assuming the commented state assignment is required
    #     new_id = [3, pen, 5, 4, 3, 5]  # queue_id, vehicle_id, origin_lane, cur_lane, MP1, MP2.....MPn
    #     new_metric = [0, 0, 0]  # time, fuel, energy, distance, MP1_dis, MP2_dis
    #     new_j = 31
    #     str = trajs[leftb[0] + 1:straightb[0]]
    #     New = new(new_lane, new_id, new_metric, new_j, str)

    # lane 6
    elif init_queue[9] == 6 and init_queue[10] == 1:
        new_lane = 6
        new_decision = 1
        # new_state = [200, init_queue[3], 0]  # Assuming the commented state assignment is required
        new_id = [1, pen, 6, 6, 5, 4, 3, 2]  # queue_id, vehicle_id, origin_lane, cur_lane, MP1, MP2.....MPn
        new_metric = [0, 0, 0]  # time, fuel, energy, distance, MP1_dis, MP2_dis
        new_j = 1
        str = trajs[straightd[0] + 1:]
        New = new(new_lane, new_id, new_metric, new_j, str, new_decision)

    # elif init_queue[9] == 6 and init_queue[10] == 2:
    #     new_lane = 6
    #     # new_state = [200, init_queue[3], 0]  # Assuming the commented state assignment is required
    #     new_id = [2, pen, 6, 7, 3, 15, 17, 20, 25]  # queue_id, vehicle_id, origin_lane, cur_lane, MP1, MP2.....MPn
    #     new_metric = [0, 0, 0]  # time, fuel, energy, distance, MP1_dis, MP2_dis
    #     new_j = 31
    #     str = trajs[leftb[0] + 1:straightb[0]]
    #     New = new(new_lane, new_id, new_metric, new_j, str)

    elif init_queue[9] == 6 and init_queue[10] == 3:
        new_lane = 6
        new_decision = 3
        # new_state = [200, init_queue[3], 0]  # Assuming the commented state assignment is required
        new_id = [3, pen, 6, 4, 1]  # queue_id, vehicle_id, origin_lane, cur_lane, MP1, MP2.....MPn
        new_metric = [0, 0, 0]  # time, fuel, energy, distance, MP1_dis, MP2_dis
        new_j = 1
        str = trajs[rightd[0] + 1:leftd[0]]
        New = new(new_lane, new_id, new_metric, new_j, str, new_decision)

    # lane 7
    # elif init_queue[9] == 7 and init_queue[10] == 1:
    #     new_lane = 7
    #     new_id = [1, pen, 7, 7, 8, 12, 16, 19, 25]  # queue_id, vehicle_id, origin_lane, cur_lane, MP1, MP2.....MPn
    #     new_metric = [0, 0, 0]  # time, fuel, energy, distance, MP1_dis, MP2_dis
    #     new_j = 31
    #     str = trajs[leftb[0] + 1:straightb[0]]
    #     New = new(new_lane, new_id, new_metric, new_j, str)

    elif init_queue[9] == 7 and init_queue[10] == 2:
        new_lane = 7
        new_decision = 2
        new_id = [2, pen, 7, 1, 4, 7]  # queue_id, vehicle_id, origin_lane, cur_lane, MP1, MP2.....MPn
        new_metric = [0, 0, 0]  # time, fuel, energy, distance, MP1_dis, MP2_dis
        new_j = 45
        str = trajs[lefta[0] + 1:straighta[0]]
        New = new(new_lane, new_id, new_metric, new_j, str, new_decision)

    # elif init_queue[9] == 7 and init_queue[10] == 3:
    #     new_lane = 7
    #     new_id = [3, pen, 7, 6, 4, 6]  # queue_id, vehicle_id, origin_lane, cur_lane, MP1, MP2.....MPn
    #     new_metric = [0, 0, 0]  # time, fuel, energy, distance, MP1_dis, MP2_dis
    #     new_j = 45
    #     str = trajs[leftb[0] + 1:straightb[0]]
    #     New = new(new_lane, new_id, new_metric, new_j, str)

    # lane 8

    elif init_queue[9] == 8 and init_queue[10] == 1:
        new_lane = 8
        new_decision = 1
        new_id = [1, pen, 8, 8, 3, 6, 8, 12]  # queue_id, vehicle_id, origin_lane, cur_lane, MP1, MP2.....MPn
        new_metric = [0, 0, 0]  # time, fuel, energy, distance, MP1_dis, MP2_dis
        new_j = 45
        str = trajs[straighta[0] + 1:originb[0]]
        New = new(new_lane, new_id, new_metric, new_j, str, new_decision)

    # elif init_queue[9] == 8 and init_queue[10] == 2:
    #     new_lane = 8
    #     new_id = [2, pen, 8, 1, 4, 8, 13, 17, 22]  # queue_id, vehicle_id, origin_lane, cur_lane, MP1, MP2.....MPn
    #     new_metric = [0, 0, 0]  # time, fuel, energy, distance, MP1_dis, MP2_dis
    #     new_j = 45
    #     str = trajs[straighta[0] + 1:originb[0]]
    #     New = new(new_lane, new_id, new_metric, new_j, str)

    elif init_queue[9] == 8 and init_queue[10] == 3:
        new_lane = 8
        new_decision = 3
        new_id = [3, pen, 8, 6, 2]  # queue_id, vehicle_id, origin_lane, cur_lane, MP1, MP2.....MPn
        new_metric = [0, 0, 0]  # time, fuel, energy, distance, MP1_dis, MP2_dis
        new_j = 45
        str = trajs[righta[0] + 1:lefta[0]]
        New = new(new_lane, new_id, new_metric, new_j, str, new_decision)

    time_set = np.zeros(6)  # Creating an array of zeros with length 6

    delta_straight = 4
    delta_left = 4
    delta_right = 2
    delta_safety = 1.8

    if New.id[0] == 1:
        New.deltaTurn = delta_straight
    elif New.id[0] == 2:
        New.deltaTurn = delta_left
    else:
        New.deltaTurn = delta_right

    lengths = find_MPs(init_queue[9], init_queue[10], trajs, New.j)

    metric = New.metric
    metric.extend([lengths[-1]])
    metric.extend(lengths.T)
    New.ocpar = np.array(OCT1(0.1 * i, init_queue[6], New.metric[3]), dtype=float)
    New.prestate = np.array([-1, - 1, - 1])
    New.phiLateral = 1.8
    New.phiRearEnd = 0.9
    New.k_lateral = 0.5 * np.ones(5)
    New.k_rear = 0.5
    New.carlength = 3.47


    if New.metric[4] == -1:
        index = search_i_p(car['que1'], new)
        if len(index) == 0:
            New.metric[4] = upperLaneChanging
        else:
            New.metric[4] = get_L_1(0.1 * i, New.ocpar, car['que1'][index].ocpar)

        if New.metric[4] < lowerLaneChanging:
            New.metric[4] = lowerLaneChanging

        if New.metric[4] > upperLaneChanging:
            New.metric[4] = upperLaneChanging

    car['cars1'] += 1

    preinitial_xycoor = New.str[New.j].split(' ')
    preinitial_xcoor = preinitial_xycoor[0]
    preinitial_xcoor = float(preinitial_xcoor.replace("x=", ''))
    preinitial_ycoor = preinitial_xycoor[2]
    preinitial_ycoor = float(preinitial_ycoor.replace("y=", ''))

    initial_xycoor = New.str[New.j + 1].split(' ')
    initial_xcoor = initial_xycoor[0]
    initial_xcoor = float(initial_xcoor.replace("x=", ''))
    initial_ycoor = initial_xycoor[2]
    initial_ycoor = float(initial_ycoor.replace("y=", ''))

    New.realpose = [initial_xcoor, initial_ycoor]
    New.prerealpose = [preinitial_xcoor, preinitial_ycoor]

    New.trust = [0, 0]
    New.see = []
    New.rearendconstraint = []
    New.lateralconstraint = np.full((5,1), np.nan)

    New.scores = [np.nan, np.nan, np.nan, np.nan]
    New.reward = 0
    New.infeasibility = 0
    New.regret = 0
    New.MUSTleave = 0
    New.agent = init_queue[7]
    New.Warning = 0
    New.NewWarning = 0
    New.overtake = 0

    car['que1'].append(
        {'state': New.state, 'prestate': New.prestate, 'realpose': New.realpose, 'prerealpose': New.prerealpose,
         'lane': New.lane, 'decision': New.decision, 'id': New.id, 'metric': New.metric, 'j': New.j, "deltaTurn": New.deltaTurn,
         "ocpar": New.ocpar, 'trust': New.trust, 'see': New.see, 'rearendconstraint': New.rearendconstraint,
         'lateralconstraint': New.lateralconstraint,
         'scores': New.scores, 'reward': New.reward, 'infeasibility': New.infeasibility, 'regret': New.regret,
         'MUSTleave': New.MUSTleave, 'k_lateral':New.k_lateral, 'k_rear': New.k_rear,
         'agent': New.agent, 'Warning': New.Warning, 'NewWarning': New.NewWarning, 'overtake': New.overtake,
         'phiRearEnd':New.phiRearEnd, "phiLateral":New.phiLateral, 'carlength': New.carlength})

    car['cars'] += 1
    pen += 1
    if pen >= 100:
        pen = 1

    return car, pen

# i = 30
# init_queue = [2,1,3,10,0,2]
# pen = (-2.0,)
# car = {'cars': 1, 'cars1': 2, 'cars2': 0, 'Car_leaves': 0, 'Car_leavesMain': 0, 'Car_leavesMerg': 0, 'que1': [{'state': [22.18577553999395, 12.160697896428081, 1.0445228979080219], 'lane': 2, 'id': [3, 1.0, 2, 8, 28], 'metric': [2.0000000000000004, 3.616223099643131, 1.1676266980683228, 622.0, 309.03207887907064], 'deltaTurn': 2, 'ocpar': np.array([-0.03770785,  1.15388122,  8.8649727 , -9.43562867, 30.60055723,
#        26.5196769 ])}], 'que2': [], 'table': [np.array([1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
#        0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 2., 0.])]}
# pointer = 2
# mod = 1
# maxTime = (-10.0,)
# CAV_oc = []
# check_arrival(i, init_queue, car, pen[0], pointer, CAV_oc, mod, maxTime)
