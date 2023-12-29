import numpy as np


def init(total, range_val):
    car = {
        'cars': 0,
        'Car_leaves': 0,
        'Car_leavesMain': 0,
        'Car_leavesMerg': 0,
        'que': [],
        'table': [],
        'order': np.empty((0), dtype=int)
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
        'acc': np.full((range_val, total+1), np.nan),
        'arrivalexit': np.zeros((total+1, 2)),
        'vel': np.full((range_val, total+1), np.nan),
        'pos': np.full((range_val, total+1), np.nan),
        'rear_end': np.full((range_val, total+1), np.nan),
        'rear_end_CBF': np.full((range_val, total+1), np.nan),
        'lateral': np.full((range_val, total+1, 5), np.nan),
        'lateral_CBF': np.full((range_val, total+1, 5), np.nan),
        'time': np.zeros((total+1, 1)),
        'fuel': np.zeros((total+1, 1)),
        'energy': np.zeros((total+1, 1)),
        'x_tk': np.empty((total+1, 8, total)),
        'v_tk': np.empty((total+1, 8, total)),
        'trust_tk': np.empty((total+1, 6)),
        'posx': np.full((range_val, total+1), np.nan),
        'posy': np.full((range_val, total+1), np.nan),
        'angle': np.full((range_val, total+1), np.nan)
    }

    for id in range(total+1):
        for i in range(0,7):
            CAV_e['x_tk'][id][i] = np.zeros(total)
            CAV_e['v_tk'][id][i] = np.zeros(total)
    for i in range(6):
        CAV_e['trust_tk'][:, i] = np.zeros(total+1)

    CAV_e['counter'] = np.zeros((total, 1))
    CAV_e['trust'] = np.zeros((range_val, total))
    CAV_e['buffer'] = np.zeros((total, 3))
    CAV_e['SpoofedCarsList'] = []
    CAV_e['SpoofedCAVsSuccedingcarlist'] = []
    CAV_e['numcollisions'] = 0

    return [car, metric, CAV_e]
