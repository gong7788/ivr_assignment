import sympy as sy
import numpy as np

green_co = [0.03948031, -1.61869256,  4.61919585]

green_X = green_co[0]
green_Y = green_co[1]
green_Z = green_co[2]

angle1, angle2, angle3 = sy.symbols("angle1 angle2 angle3")

eq = [3*sy.cos(angle3)*sy.sin(angle1)*sy.sin(angle2) + 3*sy.cos(angle1)*sy.sin(angle3) - green_X,
      -3*sy.cos(angle1)*sy.cos(angle3)*sy.sin(angle2) + 3*sy.sin(angle1)*sy.sin(angle3) - green_Y,
      2 + 3*sy.cos(angle2)*sy.cos(angle3) - green_Z]
result = sy.nonlinsolve(eq, [angle1, angle2, angle3])
print(result)