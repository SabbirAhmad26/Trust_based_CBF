def search_for_conflictCAVS_py(table, egocar):
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
