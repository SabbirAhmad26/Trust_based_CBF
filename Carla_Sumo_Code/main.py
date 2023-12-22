from scipy.io import loadmat
import numpy as np
import init
from checkArrival import check_arrival
import pandas as pd


file_path = '/home/akua/Downloads/dataset_init(2).xlsx'
data = pd.read_excel(file_path)
init_queue = data.values
total= len(init_queue)

with open('Position Values for ABCD', 'r') as file:
	trajs = file.read()
	

a = init_queue
global beta
global cnt

cnt = 0
L = 300
mode = 1

queue_cbf = np.zeros((total, 3))

car, metric = init.init()

order = []
identityOrder = []

pen = 1
pointer = 1

#import numpy as np

#def main_test(car):
#    print(car)
#    if (car['cars'] > 0):
#        #print(car)
#        order = np.zeros(len(car['que1']))
#        for j in range(len(car['que1'])):
#            order[j] = j
        #car = update_table(car, order);
#    return car
maxTime = -10
for i in range(0, (init_queue[-1][10]) * 10):
	while pointer <= total and i == init_queue[pointer][2] * 10:
		result = check_arrival(i, init_queue[pointer], car, pen, pointer, mode, trajs)
		print(result)
		car = result[0]
		pen = result[1]
		pointer += 1

