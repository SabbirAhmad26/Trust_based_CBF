def init():
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
        'order': []
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

    return [car, metric]
