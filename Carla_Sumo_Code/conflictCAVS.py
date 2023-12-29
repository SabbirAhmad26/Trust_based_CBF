
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
        for j in range(k - 1, -1, -1):
            if table[j][13] == table[k][13]:
                ip = table[j][14]
                break

        for i in range(4, len(egocar['id'])):
            flag = 0
            for j in range(k - 1, -1, -1):
                if table[j][egocar['id'][i]] > 0:
                    index.append(table[j][14])
                    position.append(table[j][egocar['id'][i]])
                    flag = 1
                    break

            if flag == 0:
                index.append(-1)
                position.append(-1)

    return ip, index, position

def search_for_conflictCAVS_trustversion(que, table, egocar, multiple_constraints, trust_threshold):
    if multiple_constraints == 1:
        trust_th = trust_threshold['high']
        # trust_th = 0.8
    else:
        trust_th = 0

    index = []
    position = []

    for i, row in enumerate(table):
        if row[0] == egocar['id'][1]:
            k = i
            break

    ip = []
    for j in range(k - 1, -1, -1):
        if table[j][13] == table[k][13]:
            ip.append(table[j][14])
            if que[ip[-1]-1]['trust'][0] >= trust_th:
                break

    for i in range(5, len(egocar['id']) + 1):
        index.append([])
        position.append([])

    for i, ego_id in enumerate(egocar['id'][4:], start=5):
        flag = 0
        for j in range(k - 1, -1, -1):
            if table[j][ego_id] > 0:
                index[i - 5].append(table[j][14])
                position[i - 5].append(table[j][ego_id])
                flag = 1
                # print(que[index[i - 5][-1]]['trust'][0],trust_th)
                if que[index[i - 5][-1]-1]['trust'][0] >= trust_th:
                    break
        if flag == 0:
            index[i - 5].append(-1)
            position[i - 5].append(-1)

    return ip, index, position


