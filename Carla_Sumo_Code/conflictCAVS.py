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
            if table[j][13] == table[k][13]:
                ip = table[j][14]
                break

        for i in range(4, len(egocar['id'])):
            flag = 0
            for j in range(k - 1, 0, -1):
                if table[j][egocar['id'][i] + 1] > 0:
                    index.append(table[j][14])
                    position.append(table[j][egocar['id'][i] + 1])
                    flag = 1
                    break

            if flag == 0:
                index.append(-1)
                position.append(-1)

    return ip, index, position



import numpy as np

data = [
    np.array([1, 0, 0, 0, 2, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1]),
    np.array([2, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 0, 2, 2]),
    np.array([3, 0, 0, 0, 0, 0, 2, 0, 0, 1, 0, 0, 0, 3, 3]),
    np.array([4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 4])
]

egocar ={'id': [3, 4, 8, 6, 2]}
print(search_for_conflictCAVS(data,egocar))
