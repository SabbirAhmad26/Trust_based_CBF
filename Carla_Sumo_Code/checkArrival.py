import math

import numpy as np

# import tmin
from OCT1_2 import OCT1
from get_L_1 import get_L_1
from search_i_p import search_i_p
from findMP import find_MPs



def check_arrival(i, init_queue, car, pen, trajs):

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


    if init_queue[9] == 1 and init_queue[10] == 2:
        new_lane = 1  # original_lane
        new_decision = 2
        new_id = [2, pen, 1, 3, 6, 4]  # action, vehicle_id, current_lane, destination_lane, MP1, MP2.....MPn
        new_metric = [0, 0, 0]  # time, fuel, energy, distance, MP1_dis, MP2_dis
        new_j = 31
        str = trajs[leftb[0] + 1:straightb[0]]
        New = new(new_lane, new_id, new_metric, new_j, str, new_decision)
        New.realpose = [init_queue[7], 0]


    # lane 2

    elif init_queue[9] == 2 and init_queue[10] == 1:
        new_lane = 2
        new_decision = 1
        new_id = [1, pen, 2, 2, 8, 9, 10, 11]  # queue_id, vehicle_id, origin_lane, cur_lane, MP1, MP2.....MPn
        new_metric = [0, 0, 0]  # time, fuel, energy, distance, MP1_dis, MP2_dis
        new_j = 31
        str = trajs[straightb[0] + 1:originc[0]]
        New = new(new_lane, new_id, new_metric, new_j, str, new_decision)
        New.realpose = [init_queue[7], 0]

    elif init_queue[9] == 2 and init_queue[10] == 3:
        new_lane = 2
        new_decision = 3
        new_id = [3, pen, 2, 8, 12]  # queue_id, vehicle_id, origin_lane, cur_lane, MP1, MP2.....MPn
        new_metric = [0, 0, 0]  # time, fuel, energy, distance, MP1_dis, MP2_dis
        new_j = 31
        str = trajs[rightb[0] + 1:leftb[0]]
        New = new(new_lane, new_id, new_metric, new_j, str, new_decision)
        New.realpose = [init_queue[7], 0]

    # lane 3

    elif init_queue[9] == 3 and init_queue[10] == 2:
        new_lane = 3
        new_decision = 2
        # new_state = [100, init_queue[3], 0]  # Assuming the commented state assignment is required
        new_id = [2, pen, 3, 5, 9, 6]  # queue_id, vehicle_id, origin_lane, cur_lane, MP1, MP2.....MPn
        new_metric = [0, 0, 0]  # time, fuel, energy, distance, MP1_dis, MP2_dis
        new_j = 31
        str = trajs[leftc[0] + 1:rightc[0]]
        New = new(new_lane, new_id, new_metric, new_j, str, new_decision)
        New.realpose = [0, init_queue[7]]

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
        New.realpose = [0, init_queue[7]]


    elif init_queue[9] == 4 and init_queue[10] == 3:
        new_lane = 4
        new_decision = 3
        # new_state = [100, init_queue[3], 0]  # Assuming the commented state assignment is required
        new_id = [3, pen, 4, 2, 11]  # queue_id, vehicle_id, origin_lane, cur_lane, MP1, MP2.....MPn
        new_metric = [0, 0, 0, ]  # time, fuel, energy, distance, MP1_dis, MP2_dis
        new_j = 31
        str = trajs[rightc[0] + 1:straightc[0]]
        New = new(new_lane, new_id, new_metric, new_j, str, new_decision)
        New.realpose = [0, init_queue[7]]

    # lane 5


    elif init_queue[9] == 5 and init_queue[10] == 2:
        new_lane = 5
        new_decision = 2
        # new_state = [200, init_queue[3], 0]  # Assuming the commented state assignment is required
        new_id = [2, pen, 5, 7, 7, 9]  # queue_id, vehicle_id, origin_lane, cur_lane, MP1, MP2.....MPn
        new_metric = [0, 0, 0]  # time, fuel, energy, distance, MP1_dis, MP2_dis
        new_j = 1
        str = trajs[leftd[0] + 1:straightd[0]]
        New = new(new_lane, new_id, new_metric, new_j, str, new_decision)
        New.realpose = [-init_queue[7], 0]

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
        New.realpose = [-init_queue[7], 0]

    elif init_queue[9] == 6 and init_queue[10] == 3:
        new_lane = 6
        new_decision = 3
        # new_state = [200, init_queue[3], 0]  # Assuming the commented state assignment is required
        new_id = [3, pen, 6, 4, 1]  # queue_id, vehicle_id, origin_lane, cur_lane, MP1, MP2.....MPn
        new_metric = [0, 0, 0]  # time, fuel, energy, distance, MP1_dis, MP2_dis
        new_j = 1
        str = trajs[rightd[0] + 1:leftd[0]]
        New = new(new_lane, new_id, new_metric, new_j, str, new_decision)
        New.realpose = [-init_queue[7], 0]

    # lane 7


    elif init_queue[9] == 7 and init_queue[10] == 2:
        new_lane = 7
        new_decision = 2
        new_id = [2, pen, 7, 1, 4, 7]  # queue_id, vehicle_id, origin_lane, cur_lane, MP1, MP2.....MPn
        new_metric = [0, 0, 0]  # time, fuel, energy, distance, MP1_dis, MP2_dis
        new_j = 45
        str = trajs[lefta[0] + 1:straighta[0]]
        New = new(new_lane, new_id, new_metric, new_j, str, new_decision)
        New.realpose = [0, -init_queue[7]]


    # lane 8

    elif init_queue[9] == 8 and init_queue[10] == 1:
        new_lane = 8
        new_decision = 1
        new_id = [1, pen, 8, 8, 3, 6, 8, 12]  # queue_id, vehicle_id, origin_lane, cur_lane, MP1, MP2.....MPn
        new_metric = [0, 0, 0]  # time, fuel, energy, distance, MP1_dis, MP2_dis
        new_j = 45
        str = trajs[straighta[0] + 1:originb[0]]
        New = new(new_lane, new_id, new_metric, new_j, str, new_decision)
        New.realpose = [0, -init_queue[7]]

    elif init_queue[9] == 8 and init_queue[10] == 3:
        new_lane = 8
        new_decision = 3
        new_id = [3, pen, 8, 6, 2]  # queue_id, vehicle_id, origin_lane, cur_lane, MP1, MP2.....MPn
        new_metric = [0, 0, 0]  # time, fuel, energy, distance, MP1_dis, MP2_dis
        new_j = 45
        str = trajs[righta[0] + 1:lefta[0]]
        New = new(new_lane, new_id, new_metric, new_j, str, new_decision)
        New.realpose = [0, -init_queue[7]]

    lengths = find_MPs(init_queue[9], init_queue[10], trajs, New.j)

    metric = New.metric
    metric.extend([lengths[-1]])
    metric.extend(lengths.T)
    New.ocpar = np.array(OCT1(0.1 * i, init_queue[6], New.metric[3]), dtype=float)
    New.prestate = np.array([-1, - 1, - 1])
    New.phiLateral = 1.8
    New.phiRearEnd = 0.9
    New.k_lateral = 0.2 * np.ones(5)
    New.k_rear_end = 0.2
    New.carlength = 3.47



    car['cars'] += 1

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

    New.realpose = [initial_xcoor + New.realpose[0], initial_ycoor+ New.realpose[1]]
    New.prerealpose = [preinitial_xcoor, preinitial_ycoor]

    New.trust = [0, 0]
    New.see = []
    New.rearendconstraint = []
    New.lateralconstraint = np.full((5,1), np.nan)
    # New.lateralconstraint = []
    New.scores = [np.nan, np.nan, np.nan, np.nan]
    New.reward = 0
    New.infeasibility = 0
    New.regret = 0
    New.mustleave = 0
    New.agent = init_queue[8]
    if New.agent == 0:
        New.traciID = init_queue[0]
    else:
        New.traciID = -1
    New.warning1 = 0
    New.warning2 = 0
    New.overtake = 0


    car['que'].append(
        {'state': New.state, 'prestate': New.prestate, 'realpose': New.realpose, 'prerealpose': New.prerealpose,
         'lane': New.lane, 'decision': New.decision, 'id': New.id, 'metric': New.metric, 'j': New.j,
         "ocpar": New.ocpar, 'trust': New.trust, 'see': New.see, 'rearendconstraint': New.rearendconstraint,
         'lateralconstraint': New.lateralconstraint, 'ip': [], 'ic': [], 'position' :[], 'traciID' : New.traciID,
         'scores': New.scores, 'reward': New.reward, 'infeasibility': New.infeasibility, 'regret': New.regret,
         'mustleave': New.mustleave, 'k_lateral': New.k_lateral, 'k_rear_end': New.k_rear_end,
         'agent': New.agent, 'warning1': New.warning1, 'warning2': New.warning2, 'overtake': New.overtake,
         'phiRearEnd':New.phiRearEnd, "phiLateral":New.phiLateral, 'carlength': New.carlength})

    pen += 1
    if pen >= 100:
        pen = 1

    return car, pen

