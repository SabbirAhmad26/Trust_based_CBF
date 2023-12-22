
import numpy as np
#import update_table_2 as update
import main_OCBF as controller

def update_table(car, order, size):
    size = len([order])
    car['table'] = []

    for j in range(size):
        temp = np.zeros(31)
        # 1st: index, 2-29: CP 1-28, 30: current lane
        # 31: order No, that is the index of the vehicle in the queue
        temp[0] = car['que1'][int(order[j])]['id'][1]
        for i in range(4, len(car['que1'][int(order[j])]['id'])):
            temp[car['que1'][int(order[j])]['id'][i] + 1] = i - 3
        temp[29] = car['que1'][int(order[j])]['id'][2]
        temp[30] = int(order[j])
        car['table'].append(temp)
    return car

def check_leave(que):
    length_que = len(que)
    index = None
    L = 300

    for i in range(length_que):
        if que[i]['state'][0] > que[i]['metric'][3]:
            index = i + 1  # MATLAB indexing starts from 1
            break

    return index

def main_CBF_update(car, metric, index, sign):
    car['Car_leaves'] += 1

    metric['Ave_time'] = ((car['Car_leaves'] - 1) * metric['Ave_time'] + car['que1'][index]['metric'][0]) / car['Car_leaves']
    metric['Ave_eng'] = ((car['Car_leaves'] - 1) * metric['Ave_eng'] + car['que1'][index]['metric'][1]) / car['Car_leaves']
    metric['Ave_u2'] = ((car['Car_leaves'] - 1) * metric['Ave_u2'] + car['que1'][index]['metric'][2]) / car['Car_leaves']
    car['cars1'] -= 1
    car['que1'].pop(index)

    t_val = str(metric['Ave_time'])
    e_val = str(metric['Ave_eng'])
    c_val = str(car['Car_leaves'])
    u2_val = str(metric['Ave_u2'])
    u2Main_val = str(metric['Ave_u2Main'])
    u2Merg_val = str(metric['Ave_u2Merg'])
    eMain_val = str(metric['Ave_engMain'])
    eMerg_val = str(metric['Ave_engMerg'])
    tMain_val = str(metric['Ave_timeMain'])
    tMerg_val = str(metric['Ave_timeMerg'])

    car['cars'] -= 1

    return car, metric

def main_test(car,i,mode):
    if (car['cars'] > 0):
        order = np.zeros(len(car['que1']))
        for j in range(len(car['que1'])):
            order[j] = j
        size = len(order)
        car = update_table(car, order, size)

    if (car['cars'] != 0):
        car = controller.main_OCBF(i, car, mode);  # calculate and update

    if (car['cars1'] > 0):
        index = check_leave(car['que1']) # check CAVs that leave the CZ in queue1
        if index:
            for k in range(len(car['table'])):
                if car['table'][k][30] == index:
                    del car['table'][k]
                    break
        
            for t in range(len(car['table'])):
                if car['table'][t][30] > index:
                    car['table'][t][30] -= 1
            car, metric = main_CBF_update(car, metric, index, 1)
            queue_cbf[car['Car_leaves'], :] = [car['Car_leaves'], metric['Ave_time'], metric['Ave_u2']]
    return car

#car = {'cars': 2, 'cars1': 4, 'cars2': 0, 'Car_leaves': 0, 'Car_leavesMain': 0, 'Car_leavesMerg': 0, 'que1': [{'state': [22.18577553999395, 12.160697896428081, 1.0445228979080219], 'lane': 2, 'id': [3, 1.0, 2, 8, 28], 'metric': [2.0000000000000004, 3.616223099643131, 1.1676266980683228, 622.0, 309.03207887907064], 'deltaTurn': 2, 'ocpar': np.array([-0.03770785,  1.15388122,  8.8649727 , -9.43562867, 30.60055723,
#           26.5196769 ])}, {'state': np.array([ 0., 10.,  0.]), 'lane': 1, 'id': [2, 2.0, 1, 3, 18, 16, 13, 9], 'metric': [0, 0, 0, 622.0, 305.96525926223745, 310.3497663547789, 309.67788681185596, 314.0623939043975], 'deltaTurn': 4, 'ocpar': np.array([ -0.03770785,   1.22929692,   6.48179456, -24.8075345 ,
#            32.60055723,  26.5196769 ])}], 'que2': [], 'table': [np.array([1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
#           0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 2., 0.])]}
#order = [1,2]
#size = 2
#car = update_table(car, order, size)
#print(car)