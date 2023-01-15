from sympy import symbols, Matrix, eye, sin, cos, pi, pprint, diff
import math
import sympy as sym
import numpy as np
import matplotlib.pyplot as plt
from functools import partial
from mpl_toolkits.mplot3d import axes3d, Axes3D



Q1, Q2, Q3, Q4, Q5= symbols('coxa femur tibia pitch wheel')
Joint_Angles = [Q1, Q2, Q3, Q4, Q5]

round_3 = partial(round, ndigits=3)
t = 0

DH_Parameter = [
    {'a': 0, 'd': 0.091, 'alpha': 0      },
    {'a': 0.191, 'd': 0, 'alpha': -pi / 2},
    {'a': 0.500, 'd': 0, 'alpha': 0      },
    {'a': 0.450, 'd': 0, 'alpha': 0      },
    {'a': 0.535, 'd': 0, 'alpha': 0      },
]

FKine = eye(4)
Jacobian_M = eye(6)
T = []
Z_M = []
O_M = []

for i, (P, Q) in enumerate(zip(DH_Parameter, Joint_Angles)):
    d = P['d']
    a = P['a']
    alpha = P['alpha']

    Transform_M = Matrix([[cos(Q), -sin(Q) * cos(alpha), sin(Q) * sin(alpha), a * cos(Q)], \
                        [sin(Q), cos(Q) * cos(alpha), -cos(Q) * sin(alpha), a * sin(Q)], \
                        [0, sin(alpha), cos(alpha), d], \
                        [0, 0, 0, 1]])

    T.append(Transform_M)
    FKine = Transform_M @ FKine

    Z_M.append(FKine[0:3, 3])
    O_M.append(FKine[0:3, 2])

T01 = T[0]
T02 = T[0] * T[1]
T03 = T[0] * T[1] * T[2]
T04 = T[0] * T[1] * T[2] * T[3]
T05 = T[0] * T[1] * T[2] * T[3] * T[4]

print("End Effector(wheel) Transformation Matrix = ")
Transform_Matrix = T05.subs({Q1: math.radians(0), Q2: math.radians(30), Q3: math.radians(30), Q4: math.radians(30), Q5: math.radians(0)}).evalf()
pprint(Transform_Matrix)
