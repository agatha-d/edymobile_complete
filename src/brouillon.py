'''
from hungarian_algorithmV2 import hungarian_algorithm2
from hungarian_algorithm import hungarian_algorithm, ans_calculation
from munkres import Munkres
import numpy as np

cost_matrix2 = np.array([[22, 24, 10, 13, 15], [18, 20, 14, 9, 11], [27, 29, 9, 18, 20], [14, 16, 8, 21, 23]])
print(cost_matrix2.shape)
assignment = hungarian_algorithm2(cost_matrix2)
print(assignment)


cost_matrix= np.array([[22, 18, 27, 14,  0],[24, 20, 29, 16,  0], [10, 14,  9,  8,  0], [13,  9, 18, 21,  0], [15, 11, 20, 23,  0]])
ans_pos = hungarian_algorithm(cost_matrix.copy())#Get the element position.
ans, ans_mat = ans_calculation(cost_matrix, ans_pos)#Get the minimum or maximum value and corresponding matrix.
result = np.nonzero(ans_mat)
print(result)

m = Munkres()
cost_matrix3= np.array([[22, 18, 27, 14],[24, 20, 29, 16], [10, 14,  9,  8], [13,  9, 18, 21], [15, 11, 20, 23]])
indexes = m.compute(cost_matrix2)
print(indexes)

tasks_to_do = ['discovery', 'omni', 'synt']

print(len(tasks_to_do))'''

stations = [0, 1 , 2, 3]

tasks_to_do = [0, 1]
next_tasks_to_do = [0, 2]

station_list = [i for i in range(0, len(stations))]

for i in range(0, 2):
      if next_tasks_to_do[i] in station_list:
        station_list.remove(next_tasks_to_do[i])

for i in range(0, 2):
        if tasks_to_do[i] == next_tasks_to_do[i]:
            print('Same task')
            next_tasks_to_do[i] = random.choice(station_list)
