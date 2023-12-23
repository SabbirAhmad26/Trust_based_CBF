from conflictCAVS import search_for_conflictCAVS

def attack_Detection(que, table, order):
    num_spoofed_cars = 0
    spoofed_cars_list = []
    spoofed_cavs_succeeding_car_list = []

    for k in range(len(order)):
        if que[order[k]]['MUSTleave'] == 1:
            num_spoofed_cars += 1
            spoofed_cars_list.append(order[k])

    order_spoofed_car_list = [order.index(car) for car in spoofed_cars_list]
    order_spoofed_car_list.append(len(order)+1)  # Actual order of the spoofed cars + the index of

    for j in range(len(spoofed_cars_list)):
        index = -1
        for jj in order[order_spoofed_car_list[j] + 1: order_spoofed_car_list[j + 1] - 1]:
            egocar = que[jj]
            ip, _, _ = search_for_conflictCAVS(table, egocar)
            if spoofed_cars_list[j] in ip:
                index = jj
                spoofed_cavs_succeeding_car_list.append(index)
                que[jj]['overtake'] = 1
                break

        if index == -1:
            spoofed_cavs_succeeding_car_list.append(index)

    return spoofed_cars_list, order_spoofed_car_list, spoofed_cavs_succeeding_car_list, que

def exp_imp_constraint_finder_mitigation(que, index_in_the_list, k, spoofed_cars_list, order, table):
    explicit_constraint_indices = k

    order_spoofed_car_list = [order.index(car) for car in spoofed_cars_list]
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


def mitigation_function(que1, table, order, CAV_e):
    spoofed_cars_list, spoofed_cavs_succeeding_car_list, que1 = attack_Detection(que1, table, order)

    for j in range(len(CAV_e['SpoofedCarsList'])):
        if CAV_e['SpoofedCAVsSuccedingcarlist'][j] != -1:
            suc_ind = CAV_e['SpoofedCAVsSuccedingcarlist'][j]
            s_ind = CAV_e['SpoofedCarsList'][j]
            if que1[s_ind]['state'][0] > que1[suc_ind]['state'][0]:
                spoofed_cars_list.pop(j)
                spoofed_cavs_succeeding_car_list.pop(j)
    
    for j in range(len(spoofed_cars_list)):
        if spoofed_cavs_succeeding_car_list[j] != -1:
            explicit_constraint_indices = exp_imp_constraint_finder_mitigation(
                que1, spoofed_cavs_succeeding_car_list[j], spoofed_cars_list[j], table
            )

            ind1 = order.index(spoofed_cars_list[j])
            ind2 = order.index(spoofed_cavs_succeeding_car_list[j])

            set1 = order[ind1 + 1: ind2]
            sj = explicit_constraint_indices
            set2 = order[ind2 + 1:]

            set2 = list(set(set2) ^ set(set2).intersection(sj))
            set2 = set2 + [ind2]

            CAV_e['SpoofedCarsList'][j] = spoofed_cars_list[j]
            CAV_e['SpoofedCAVsSuccedingcarlist'][j] = spoofed_cavs_succeeding_car_list[j]

        else:
            CAV_e['SpoofedCarsList'][j] = spoofed_cars_list[j]
            CAV_e['SpoofedCAVsSuccedingcarlist'][j] = spoofed_cavs_succeeding_car_list[j]
            explicit_constraint_indices = []
            ind1 = order.index(spoofed_cars_list[j])
            set1 = order[ind1 + 1:]
            ind2 = None
            set2 = []

        order = order[:ind1] + set1 + set2 + [ind2] + [ind1] + explicit_constraint_indices

    return order
