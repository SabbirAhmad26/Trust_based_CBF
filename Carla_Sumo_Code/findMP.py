import numpy as np


def find_MPs(origin, exit, trajs, j):
    lengths = np.empty([0])
    distance = 0
    flag1 = True
    flag2 = True
    flag3 = True
    flag4 = True

    locations = np.array([[288.92, 204.819],
                          [295.374, 187.581],
                          [312.595, 195.049],
                          [305.7, 213.316],
                          [295.257, 205.089],
                          [295.331, 194.567],
                          [305.829, 194.951],
                          [305.755, 205.462],
                          [300.536, 205.263],
                          [295.614, 199.744],
                          [300.52, 194.9],
                          [305.634, 200.307]])

    righta = [i for i, line in enumerate(trajs) if line == 'Right A:']
    lefta = [i for i, line in enumerate(trajs) if line == 'Left A:']
    straighta = [i for i, line in enumerate(trajs) if line == 'straight A:']

    originb = [i for i, line in enumerate(trajs) if line == 'Origin Lane B:']
    rightb = [i for i, line in enumerate(trajs) if line == 'Right B:']
    leftb = [i for i, line in enumerate(trajs) if line == 'Left B:']
    straightb = [i for i, line in enumerate(trajs) if line == 'Straight B:' or line == 'Straight B: ']

    originc = [i for i, line in enumerate(trajs) if line == 'Origin Lane C:']
    rightc = [i for i, line in enumerate(trajs) if line == 'Right C:']
    leftc = [i for i, line in enumerate(trajs) if line == 'Left C:']
    straightc = [i for i, line in enumerate(trajs) if line == 'Straight C:']

    origind = [i for i, line in enumerate(trajs) if line == 'Origin Lane D:' or line == 'Origin Lane D: ']
    rightd = [i for i, line in enumerate(trajs) if line == 'Right D:' or line == 'Right D: ']
    leftd = [i for i, line in enumerate(trajs) if line == 'Left D:']
    straightd = [i for i, line in enumerate(trajs) if line == 'Straight D']

    if (origin == 8 and exit == 3):
        str = str = trajs[righta[0] + 1:lefta[0]]
        temp = str[j].split(' ')
        temp_x = float(temp[0].replace("x=", ''))
        temp_y = float(temp[2].replace("y=", ''))

        for i in range(j, len(str)):
            temp = str[i].split(' ')
            x_info = float(temp[0].replace("x=", ''))
            y_info = float(temp[2].replace("y=", ''))
            distance = distance + np.sqrt((temp_x - x_info) ** 2 + (temp_y - y_info) ** 2)
            if (x_info <= locations[0, 0] and flag1):
                lengths = np.append(lengths, distance)
                flag1 = False

            temp_x = x_info
            temp_y = y_info

    elif (origin == 8 and exit == 1):
        str = str = trajs[straighta[0] + 1:originb[0]]
        temp = str[j].split(' ')
        temp_x = float(temp[0].replace("x=", ''))
        temp_y = float(temp[2].replace("y=", ''))

        for i in range(j, len(str)):
            temp = str[i].split(' ')
            x_info = float(temp[0].replace("x=", ''))
            y_info = float(temp[2].replace("y=", ''))
            distance = distance + np.sqrt((temp_x - x_info) ** 2 + (temp_y - y_info) ** 2)

            if (y_info <= locations[4, 1] and flag1):
                lengths = np.append(lengths, distance)
                flag1 = False
            elif (y_info <= locations[9, 1] and flag2):
                lengths = np.append(lengths, distance)
                flag2 = False
            elif (y_info <= locations[5, 1] and flag3):
                lengths = np.append(lengths, distance)
                flag3 = False
            elif (y_info <= locations[1, 1] and flag4):
                lengths = np.append(lengths, distance)
                flag4 = False

            temp_x = x_info
            temp_y = y_info

    elif (origin == 7 and exit == 2):
        str = str = trajs[lefta[0] + 1:straighta[0]]
        temp = str[j].split(' ')
        temp_x = float(temp[0].replace("x=", ''))
        temp_y = float(temp[2].replace("y=", ''))

        for i in range(j, len(str)):
            temp = str[i].split(' ')
            x_info = float(temp[0].replace("x=", ''))
            y_info = float(temp[2].replace("y=", ''))
            distance = distance + np.sqrt((temp_x - x_info) ** 2 + (temp_y - y_info) ** 2)

            if (y_info <= locations[8, 1] and flag1):
                lengths = np.append(lengths, distance)
                flag1 = False
            elif (x_info >= locations[11, 0] and flag2):
                lengths = np.append(lengths, distance)
                flag2 = False

            temp_x = x_info
            temp_y = y_info

    elif (origin == 2 and exit == 3):
        str = str = trajs[rightb[0] + 1:leftb[0]]
        temp = str[j].split(' ')
        temp_x = float(temp[0].replace("x=", ''))
        temp_y = float(temp[2].replace("y=", ''))

        for i in range(j, len(str)):
            temp = str[i].split(' ')
            x_info = float(temp[0].replace("x=", ''))
            y_info = float(temp[2].replace("y=", ''))
            distance = distance + np.sqrt((temp_x - x_info) ** 2 + (temp_y - y_info) ** 2)

            if (y_info <= locations[1, 1] and flag1):
                lengths = np.append(lengths, distance)
                flag1 = False

            temp_x = x_info
            temp_y = y_info

    elif (origin == 2 and exit == 1):
        str = trajs[straightb[0] + 1:originc[0]]
        temp = str[j].split(' ')
        temp_x = float(temp[0].replace("x=", ''))
        temp_y = float(temp[2].replace("y=", ''))

        for i in range(j, len(str)):
            temp = str[i].split(' ')
            x_info = float(temp[0].replace("x=", ''))
            y_info = float(temp[2].replace("y=", ''))
            distance = distance + np.sqrt((temp_x - x_info) ** 2 + (temp_y - y_info) ** 2)

            if (x_info >= locations[5, 0] and flag1):
                lengths = np.append(lengths, distance)
                flag1 = False
            elif (x_info >= locations[10, 0] and flag2):
                lengths = np.append(lengths, distance)
                flag2 = False
            elif (x_info >= locations[6, 0] and flag3):
                lengths = np.append(lengths, distance)
                flag3 = False
            elif (x_info >= locations[2, 0] and flag4):
                lengths = np.append(lengths, distance)
                flag4 = False

            temp_x = x_info
            temp_y = y_info

    elif (origin == 1 and exit == 2):
        str = str = trajs[leftb[0] + 1:straightb[0]]
        temp = str[j].split(' ')
        temp_x = float(temp[0].replace("x=", ''))
        temp_y = float(temp[2].replace("y=", ''))

        for i in range(j, len(str)):
            temp = str[i].split(' ')
            x_info = float(temp[0].replace("x=", ''))
            y_info = float(temp[2].replace("y=", ''))
            distance = distance + np.sqrt((temp_x - x_info) ** 2 + (temp_y - y_info) ** 2)

            if (y_info >= locations[8, 1] and flag1):
                lengths = np.append(lengths, distance)
                flag1 = False
            elif (y_info >= locations[9, 1] and flag2):
                lengths = np.append(lengths, distance)
                flag2 = False

            temp_x = x_info
            temp_y = y_info

    elif (origin == 4 and exit == 3):
        str = str = trajs[rightc[0] + 1:straightc[0]]
        temp = str[j].split(' ')
        temp_x = float(temp[0].replace("x=", ''))
        temp_y = float(temp[2].replace("y=", ''))

        for i in range(j, len(str)):
            temp = str[i].split(' ')
            x_info = float(temp[0].replace("x=", ''))
            y_info = float(temp[2].replace("y=", ''))
            distance = distance + np.sqrt((temp_x - x_info) ** 2 + (temp_y - y_info) ** 2)

            if (x_info >= locations[2, 0] and flag1):
                lengths = np.append(lengths, distance)
                flag1 = False

            temp_x = x_info
            temp_y = y_info

    elif (origin == 4 and exit == 1):
        str = str = trajs[straightc[0] + 1:origind[0]]
        temp = str[j].split(' ')
        temp_x = float(temp[0].replace("x=", ''))
        temp_y = float(temp[2].replace("y=", ''))

        for i in range(j, len(str)):
            temp = str[i].split(' ')
            x_info = float(temp[0].replace("x=", ''))
            y_info = float(temp[2].replace("y=", ''))
            distance = distance + np.sqrt((temp_x - x_info) ** 2 + (temp_y - y_info) ** 2)

            if (y_info >= locations[6, 1] and flag1):
                lengths = np.append(lengths, distance)
                flag1 = False
            elif (y_info >= locations[11, 1] and flag2):
                lengths = np.append(lengths, distance)
                flag2 = False
            elif (y_info >= locations[7, 1] and flag3):
                lengths = np.append(lengths, distance)
                flag3 = False
            elif (y_info >= locations[3, 1] and flag4):
                lengths = np.append(lengths, distance)
                flag4 = False

            temp_x = x_info
            temp_y = y_info

    elif (origin == 3 and exit == 2):
        str = str = trajs[leftc[0] + 1:rightc[0]]
        temp = str[j].split(' ')
        temp_x = float(temp[0].replace("x=", ''))
        temp_y = float(temp[2].replace("y=", ''))

        for i in range(j, len(str)):
            temp = str[i].split(' ')
            x_info = float(temp[0].replace("x=", ''))
            y_info = float(temp[2].replace("y=", ''))
            distance = distance + np.sqrt((temp_x - x_info) ** 2 + (temp_y - y_info) ** 2)

            if (x_info <= locations[10, 0] and flag1):
                lengths = np.append(lengths, distance)
                flag1 = False
            elif (x_info <= locations[9, 0] and flag2):
                lengths = np.append(lengths, distance)
                flag2 = False

            temp_x = x_info
            temp_y = y_info

    elif (origin == 6 and exit == 3):
        str = str = trajs[rightd[0] + 1:leftd[0]]
        temp = str[j].split(' ')
        temp_x = float(temp[0].replace("x=", ''))
        temp_y = float(temp[2].replace("y=", ''))

        for i in range(j, len(str)):
            temp = str[i].split(' ')
            x_info = float(temp[0].replace("x=", ''))
            y_info = float(temp[2].replace("y=", ''))
            distance = distance + np.sqrt((temp_x - x_info) ** 2 + (temp_y - y_info) ** 2)

            if (y_info >= locations[3, 1] and flag1):
                lengths = np.append(lengths, distance)
                flag1 = False

            temp_x = x_info
            temp_y = y_info

    elif (origin == 6 and exit == 1):
        str = trajs[straightd[0] + 1:]
        temp = str[j].split(' ')
        temp_x = float(temp[0].replace("x=", ''))
        temp_y = float(temp[2].replace("y=", ''))

        for i in range(j, len(str)):
            temp = str[i].split(' ')
            x_info = float(temp[0].replace("x=", ''))
            y_info = float(temp[2].replace("y=", ''))
            distance = distance + np.sqrt((temp_x - x_info) ** 2 + (temp_y - y_info) ** 2)

            if (x_info <= locations[7, 0] and flag1):
                lengths = np.append(lengths, distance)
                flag1 = False
            elif (x_info <= locations[8, 0] and flag2):
                lengths = np.append(lengths, distance)
                flag2 = False
            elif (x_info <= locations[4, 0] and flag3):
                lengths = np.append(lengths, distance)
                flag3 = False
            elif (x_info <= locations[0, 0] and flag4):
                lengths = np.append(lengths, distance)
                flag4 = False

            temp_x = x_info
            temp_y = y_info

    elif (origin == 5 and exit == 2):
        str = str = trajs[leftd[0] + 1:straightd[0]]
        temp = str[j].split(' ')
        temp_x = float(temp[0].replace("x=", ''))
        temp_y = float(temp[2].replace("y=", ''))

        for i in range(j, len(str)):
            temp = str[i].split(' ')
            x_info = float(temp[0].replace("x=", ''))
            y_info = float(temp[2].replace("y=", ''))
            distance = distance + np.sqrt((temp_x - x_info) ** 2 + (temp_y - y_info) ** 2)

            if (y_info <= locations[11, 1] and flag1):
                lengths = np.append(lengths, distance)
                flag1 = False
            elif (y_info <= locations[10, 1] and flag2):
                lengths = np.append(lengths, distance)
                flag2 = False

            temp_x = x_info
            temp_y = y_info
    return lengths
