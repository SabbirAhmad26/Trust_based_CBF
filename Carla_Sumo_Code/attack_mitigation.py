from conflictCAVS import search_for_conflictCAVS
import numpy as np

def find_index(lst, value):
    try:
        index = lst.index(value)
        return index
    except ValueError:
        return -1  # Value not found in the list


def attack_detection(que, table, order):
    num_spoofed_cars = 0
    spoofed_cars_list = []
    spoofed_cars_succeeding_cavs_list = []

    for k in order:
        if que[k - 1]['mustleave'] == 1:
            num_spoofed_cars += 1
            spoofed_cars_list.append(k-1)

    #order_spoofed_car_list = [np.where(order == car)[0] for car in spoofed_cars_list]
    order_spoofed_car_list = [index + 1  for index in spoofed_cars_list]
    order_spoofed_car_list.append(len(order) + 1)

    for j in range(len(spoofed_cars_list)):
        index = -1
        for jj in order[order_spoofed_car_list[j]: order_spoofed_car_list[j + 1] - 1]:
            egocar = que[jj-1]
            ip, _, _ = search_for_conflictCAVS(table, egocar)
            if spoofed_cars_list[j] == ip-1:
                index = jj - 1
                spoofed_cars_succeeding_cavs_list.append(index)
                # que[jj]['overtake'] = 1
                break
        if index == -1:
            spoofed_cars_succeeding_cavs_list.append(index)

    return spoofed_cars_list, order_spoofed_car_list, spoofed_cars_succeeding_cavs_list #, que

def exp_imp_constraint_finder_mitigation_2(que, index_in_the_list, k, spoofed_cars_list, order, table):
    explicit_constraint_indices = k

    order_spoofed_car_list = [order[car] for car in spoofed_cars_list]
    order_spoofed_car_list.append(len(order))  # Actual order of the spoofed cars + the index of

    for j in range(len(explicit_constraint_indices)):
        for i in order[order_spoofed_car_list[index_in_the_list] + 1: order_spoofed_car_list[index_in_the_list + 1] - 1]:
            if i == explicit_constraint_indices[j]:
                continue

            egocar = que[i]
            ip, index, _ = search_for_conflictCAVS(table, egocar)

            if len(set(ip) & set(explicit_constraint_indices)) or len(set(index) & set(explicit_constraint_indices)):
                explicit_constraint_indices.append(i)

    return explicit_constraint_indices[1:]

def exp_imp_constraint_finder_mitigation(que, index_in_the_list, k, order_spoofed_car_list, order, table):

    explicit_constraint_indices = [k]  # k is the index of the succeeding car in arrival order

    # order_spoofed_car_list = [np.where(order == car)[0][0] for car in spoofed_cars_list]
    # order_spoofed_car_list.append(len(order) + 1)

    for j in range(len(explicit_constraint_indices)):
        for i in order[order_spoofed_car_list[index_in_the_list]: order_spoofed_car_list[index_in_the_list + 1] - 1]:
            if i-1 == explicit_constraint_indices[j]:
                continue

            egocar = que[i-1]
            ip, index, _ = search_for_conflictCAVS(table, egocar)
            #ip = egocar['ip']
            #index = ego['index']
            ip = ip - 1
            index = [ind - 1 for ind in index]
            if np.any(np.isin(ip, explicit_constraint_indices)) or np.any(np.isin(index, explicit_constraint_indices)):
                explicit_constraint_indices.append(i-1)

    explicit_constraint_indices = explicit_constraint_indices[1:]

    return explicit_constraint_indices


import numpy as np
def mitigation_function(que, table, order, CAV_e):
    spoofed_cars_list, order_spoofed_car_list, spoofed_cars_succeeding_cavs_list = attack_detection(que, table, order)

    # for j in range(len(CAV_e['SpoofedCarsList'])):
    #     if CAV_e['SpoofedCAVsSuccedingcarlist'][j] != -1:
    #         suc_ind = CAV_e['SpoofedCAVsSuccedingcarlist'][j]
    #         s_ind = CAV_e['SpoofedCarsList'][j]
    #         if que[s_ind]['state'][0] > que[suc_ind]['state'][0]:
    #             spoofed_cars_list[j] = -1
    #             spoofed_cavs_succeeding_carlist[j] = -1
    #
    for j in range(len(spoofed_cars_list)):
        if spoofed_cars_succeeding_cavs_list[j] != -1:
            explicit_constraint_indices = exp_imp_constraint_finder_mitigation(que, j,
                                                                               spoofed_cars_succeeding_cavs_list[j],
                                                                               order_spoofed_car_list, order, table)
            print(explicit_constraint_indices)
            ind1 = [np.where(order == order_spoofed_car_list[j])[0][0]]
            ind2 = [np.where(order == spoofed_cars_succeeding_cavs_list[j]+1)[0][0]]
            set1 = order[ind1[0] + 1:ind2[0]]
            Sj = explicit_constraint_indices
            set2 = order[ind2[0] + 1:]
            set2 = np.setxor1d(set2, np.intersect1d(set2, Sj))
        #     CAV_e['SpoofedCarsList'][j] = spoofed_cars_list[j]
        #     CAV_e['SpoofedCAVsSuccedingcarlist'][j] = spoofed_cars_succeeding_cavs_list[j]
        else:
            # CAV_e['SpoofedCarsList'][j] = spoofed_cars_list[j]
            # CAV_e['SpoofedCAVsSuccedingcarlist'][j] = spoofed_cars_succeeding_cavs_list[j]
            explicit_constraint_indices = np.array([], dtype =int)
            ind1 = [np.where(order == order_spoofed_car_list[j])[0][0]]
            set1 = order[ind1[0] + 1:]
            ind2 = np.array([], dtype = int)
            set2 = np.array([], dtype = int)
        #
        temp = order
        order = np.concatenate([order[:ind1[0]], set1, set2, np.array(order[ind2]), np.array(order[ind1]), explicit_constraint_indices])
        # if temp != order:
        #     stop = 0
        order = np.array([int(elm) for elm in order])
        # print(order)
    return order
