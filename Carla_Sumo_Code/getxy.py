import math


def getXY(origin, exit, distancecurrent, distance, j, realpose,prerealpose):

    with open('Position Values for ABCD', 'r') as file:
        trajs = file.read()
    
    trajs = trajs.splitlines()
    trajs = list(filter(None, trajs))
    
    righta = [i for i, line in enumerate(trajs) if line == 'Right A:']
    lefta = [i for i, line in enumerate(trajs) if line == 'Left A:']
    straighta = [i for i, line in enumerate(trajs) if line == 'straight A:']
    
    originb = [i for i, line in enumerate(trajs) if line == 'Origin Lane B:']
    rightb = [i for i, line in enumerate(trajs) if line == 'Right B:']
    leftb = [i for i, line in enumerate(trajs) if line == 'Left B:']
    straightb = [i for i,line in enumerate(trajs) if line == 'Straight B:' or line == 'Straight B: ']
    
    originc = [i for i, line in enumerate(trajs) if line == 'Origin Lane C:']
    rightc = [i for i, line in enumerate(trajs) if line == 'Right C:']
    leftc = [i for i, line in enumerate(trajs) if line == 'Left C:']
    straightc = [i for i, line in enumerate(trajs) if line == 'Straight C:']
    
    origind = [i for i, line in enumerate(trajs) if line == 'Origin Lane D:' or line == 'Origin Lane D: ']
    rightd = [i for i, line in enumerate(trajs) if line == 'Right D:' or line == 'Right D: ']
    leftd = [i for i, line in enumerate(trajs) if line == 'Left D:']
    straightd = [i for i, line in enumerate(trajs) if line == 'Straight D']
    
    #3 = right 1 = straight 2 = left
    if (origin == 8 and exit == 3):
        str = trajs[righta[0]+1:lefta[0]]
    elif (origin == 8 and exit == 1):
        str = trajs[straighta[0]+1:originb[0]]
    elif(origin == 7 and exit ==2):
        str = trajs[lefta[0]+1:straighta[0]]
    elif(origin == 2 and exit==3):
        str = trajs[rightb[0]+1:leftb[0]]
    elif (origin == 2 and exit==1):
        str = trajs[straightb[0]+1:originc[0]]
    elif (origin == 1 and exit==2):
        str = trajs[leftb[0]+1:straightb[0]]
    elif(origin == 4 and exit==3):
        str = trajs[rightc[0]+1:straightc[0]]
    elif (origin == 4 and exit==1):
        str = trajs[straightc[0]+1:origind[0]]
    elif (origin == 2 and exit==2):
        str = trajs[leftc[0]+1:rightc[0]]
    elif (origin == 6 and exit==3):
        str = trajs[rightd[0]+1:leftd[0]]
    elif (origin == 6 and exit==1):
        str = trajs[straightd[0]+1:]
    elif (origin == 5 and exit==2):
        str = trajs[leftd[0]+1:straightd[0]]
    elif (origin == 3 and exit==2):
        str = trajs[leftc[0] + 1:rightc[0]]


    tempx = realpose[0]
    tempy = realpose[1]
    temp = str[j].split(' ')
    angle_info = temp[3]
    tempangle = float(angle_info.replace("angle=", ''))
    
    for i in range(j+1, len(str)):
        temp = str[i].split(' ')
        x_info = temp[0]
        y_info = temp[2]
        angle_info = temp[3]
        
        x_info = float(x_info.replace("x=", ''))
        y_info = float(y_info.replace("y=", ''))
        angle_info = float(angle_info.replace("angle=", ''))
        
        if distance + ((x_info - tempx) ** 2 + (y_info - tempy) ** 2) ** 0.5 > distancecurrent:
            j = i-1
            break
    
    t = (distancecurrent - distance) / ((x_info - tempx) ** 2 + (y_info - tempy) ** 2) ** 0.5
    xinfo = (1 - t) * tempx + t * x_info
    yinfo = (1 - t) * tempy + t * y_info
    angle_rad = math.atan2(realpose[0] - prerealpose[0], realpose[1] - prerealpose[1])
    angle_deg = math.degrees(angle_rad)
    angle_info = (angle_deg + 360) % 360
    
    return xinfo, yinfo, angle_info, j

##Example
