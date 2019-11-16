import sympy as sy
import numpy as np
from scipy.optimize import least_squares
import math

# green_co = [0.03948031, -1.61869256,  4.61919585]
#
# green_X = green_co[0]
# green_Y = green_co[1]
# green_Z = green_co[2]
#
# angle1, angle2, angle3 = sy.symbols("angle1 angle2 angle3")
#
# eq = [3*sy.cos(angle3)*sy.sin(angle1)*sy.sin(angle2) + 3*sy.cos(angle1)*sy.sin(angle3) - green_X,
#       -3*sy.cos(angle1)*sy.cos(angle3)*sy.sin(angle2) + 3*sy.sin(angle1)*sy.sin(angle3) - green_Y,
#       2 + 3*sy.cos(angle2)*sy.cos(angle3) - green_Z]
# result = sy.nonlinsolve(eq, [angle1, angle2, angle3])
# print(result)
def fun_trans(theta):
    return np.array(
        [[3 * np.cos(theta[2]) * np.sin(theta[0]) * np.sin(theta[1]) + 3 * np.cos(theta[0]) * np.sin(theta[2])],
         [-3 * np.cos(theta[0]) * np.cos(theta[2]) * np.sin(theta[1]) + 3 * np.sin(theta[0]) * np.sin(theta[2])],
         [2 + 3 * np.cos(theta[1]) * np.cos(theta[2])],
         [2 + 3 * np.cos(theta[1]) * np.cos(theta[2]) + 2 * np.cos(theta[1]) * np.cos(theta[2]) * np.cos(theta[3]) - 2 * np.sin(theta[1]) * np.sin(theta[3])]])

def error(theta, true):
    temp = fun_trans(theta)
    estimated = np.array([temp[0][0], temp[1][0], temp[2][0], temp[3][0]])
    # print("estimated: ",estimated)
    # print("\n green: ", green)
    # print(temp)
    # print(estimated)
    return np.sum(((estimated-true)**2))



green = [ 2.3241, -0.7239,  3.7338]
red = [ 3.81  , -2.8575,  3.7338]

true = np.array([[green[0], green[1], green[2], red[2]]])
np.set_printoptions(suppress=True)
res_1 = least_squares(error, x0=[0, 0, 0, 0], bounds=([-math.pi, -math.pi / 2, -math.pi / 2, -math.pi / 2], [math.pi, math.pi / 2, math.pi / 2, math.pi / 2]), args=true)
print(res_1.x)
