def search_for_conflictCAVS(table, egocar):
    index = []
    position = []

    k = None
    for i in range(len(table)):
        if table[i][0] == egocar['id'][1]:
            k = i
            break

    ip = -1
    if k is not None:
        for j in range(k - 1, 0, -1):
            if table[j][29] == table[k][29]:
                ip = table[j][30]
                break

        for i in range(4, len(egocar['id'])):
            flag = 0
            for j in range(k - 1, 0, -1):
                if table[j][egocar['id'][i] + 1] > 0:
                    index.append(table[j][30])
                    position.append(table[j][egocar['id'][i] + 1])
                    flag = 1
                    break

            if flag == 0:
                index.append(-1)
                position.append(-1)

    return ip, index, position

def search_for_conflictCAVS_trustversion(que, table, order, k, egocar, multiple_constraints, trust_threshold):
    if multiple_constraints == 1:
        trust_th = trust_threshold['high']
    else:
        trust_th = 0

    index = []
    position = []

    for i, row in enumerate(table):
        if row[0] == egocar['id'][1]:
            k = i
            break

    ip = []
    for j in range(k - 1, 0, -1):
        if table[j][29] == table[k][29]:
            ip.append(table[j][30])
            if que[ip[-1]]['trust'][0] >= trust_th:
                break

    for i in range(5, len(egocar['id']) + 1):
        index.append([])
        position.append([])

    for i, ego_id in enumerate(egocar['id'][4:], start=5):
        flag = 0
        for j in range(k - 1, 0, -1):
            if table[j][ego_id] > 0:
                index[i - 5].append(table[j][30])
                position[i - 5].append(table[j][ego_id])
                flag = 1
                if que[index[i - 5][-1]]['trust'][0] >= trust_th:
                    break
        if flag == 0:
            index[i - 5].append(-1)
            position[i - 5].append(-1)

    return ip, index, position


# Example usage
que_data = [{}, {}, {}, {}]  # Replace with your actual data
table_data = None  # Replace with your actual data
order_data = None  # Replace with your actual data
k_data = None  # Replace with your actual data
egocar_data = {
    'id': [0, 9],  # Replace with your actual data
}
multiple_constraints_data = 0  # Replace with your actual data
trust_threshold_data = {'high': 0.2}  # Replace with your actual data

ip_result, index_result, position_result = search_for_conflictCAVS_trustversion(que_data, table_data, order_data,
                                                                                k_data, egocar_data,
                                                                                multiple_constraints_data,
                                                                                trust_threshold_data)
print("IP:", ip_result)
print("Index:", index_result)
print("Position:", position_result)
