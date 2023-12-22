import numpy as np


def init(total, range):
    car = {
        'cars': 0,
        'cars1': 0,
        'cars2': 0,
        'Car_leaves': 0,
        'Car_leavesMain': 0,
        'Car_leavesMerg': 0,
        'que1': [],
        'que2': [],
        'table': [],
        'order': np.empty(0)
    }

    metric = {
        'Ave_time': 0,
        'Ave_u2': 0,
        'Ave_eng': 0,
        'Ave_u2Main': 0,
        'Ave_u2Merg': 0,
        'Ave_engMain': 0,
        'Ave_engMerg': 0,
        'Ave_timeMain': 0,
        'Ave_timeMerg': 0
    }

    CAV_e = {
        'acc': np.zeros((range_val, total)),
        'arrivalexit': np.zeros((total, 2)),
        'vel': np.zeros((range_val, total)),
        'pos': np.zeros((range_val, total)),
        'rear_end': np.full((range_val, total), np.nan),
        'rear_end_CBF': np.full((range_val, total), np.nan),
        'lateral': np.full((range_val, total, 5), np.nan),
        'lateral_CBF': np.full((range_val, total, 5), np.nan),
        'time': np.zeros((total, 1)),
        'fuel': np.zeros((total, 1)),
        'energy': np.zeros((total, 1)),
        'x_tk': np.empty((total, 8), dtype=object),
        'v_tk': np.empty((total, 8), dtype=object),
        'trust_tk': np.empty((total, 6), dtype=object),
        'posx': np.zeros((range_val, total)),
        'posy': np.zeros((range_val, total)),
        'angle': np.zeros((range_val, total))
    }

    for i in range(8):
        for j in range(total):
            CAV_e['x_tk'][j, i] = np.zeros(total)
            CAV_e['v_tk'][j, i] = np.zeros(total)

    for i in range(6):
        for j in range(total):
            CAV_e['trust_tk'][j, i] = np.zeros(total)

    CAV_e['counter'] = np.zeros((total, 1))
    CAV_e['trust'] = np.zeros((range_val, total))
    CAV_e['buffer'] = np.zeros((total, 3))
    CAV_e['SpoofedCarsList'] = []
    CAV_e['SpoofedCAVsSuccedingcarlist'] = []
    CAV_e['numcollisions'] = 0

    return [car, metric, CAV_e]
