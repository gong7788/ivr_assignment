import numpy as np
import sys

np.set_printoptions(suppress=True)
theta_1 = 0
theta_2 = 0
theta_3 = 0
theta_4 = 1

theta = [1.57+theta_1, 1.57+theta_2, theta_3, -theta_4]
a = [0,0,3,2]
alpha = [1.57, 1.57, 1.57, 0]
d = [2, 0, 0, 0]


Transform = []
for i in range(4):
    T_theta = np.array([[np.cos(theta[i]), -np.sin(theta[i]), 0, 0],
                       [np.sin(theta[i]), np.cos(theta[i]), 0, 0],
                       [0, 0, 1, d[i]],
                       [0, 0, 0, 1]])

    T_alpha = np.array([[1, 0, 0,  a[i]],
                       [0, np.cos(alpha[i]), -np.sin(alpha[i]), 0],
                       [0, np.sin(alpha[i]), np.cos(alpha[i]), 0],
                       [0, 0, 0, 1]])


    Transform.append(T_theta.dot(T_alpha))

red_co = Transform[0].dot(Transform[1]).dot(Transform[2]).dot(Transform[3])[0:3, -1]
green_co = Transform[0].dot(Transform[1]).dot(Transform[2])[0:3, -1]
blue_co = Transform[0][0:3, -1]

print(red_co)
print(green_co)
print(blue_co)