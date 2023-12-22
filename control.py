import numpy as np


def update_table(car, order):
    size = len([order])
    car['table'] = []
    print(order)
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
