# -*- coding: utf-8 -*-
"""An Introduction to Robotics, Chapter3.ipynb

Automatically generated by Colaboratory.

Original file is located at
    https://colab.research.google.com/drive/1RLI0E4KVbwhQEDLm5wO2awykG96ra8PE

# An Introduction to Robotics, Chapter 3
By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
Copyright ARAS @2023

[![GitHub Repository](https://img.shields.io/badge/GitHub-Repository-blue.svg)](https://github.com/aras-labs/Robotics/)

## This code provides the forward kinematic solution of the 3R robot with DH and Screw methods:
"""

import sympy as sp

# DH function
def DH(a, alpha, d, theta):
    # Construct the Denavit-Hartenberg (DH) transformation matrix
    T = sp.Matrix([[sp.cos(theta), -sp.sin(theta) * sp.cos(alpha), sp.sin(theta) * sp.sin(alpha), a * sp.cos(theta)],
                   [sp.sin(theta), sp.cos(theta) * sp.cos(alpha), -sp.cos(theta) * sp.sin(alpha), a * sp.sin(theta)],
                   [0, sp.sin(alpha), sp.cos(alpha), d],
                   [0, 0, 0, 1]])
    return T

# SR function
def SR(s_x, s_y, s_z, s_ox, s_oy, s_oz, theta, t):
    # Construct the Screw-based transformation matrix
    S = sp.Matrix([
        [(s_x**2 - 1) * (1 - sp.cos(theta)) + 1,
         s_x * s_y * (1 - sp.cos(theta)) - s_z * sp.sin(theta),
         s_x * s_z * (1 - sp.cos(theta)) + s_y * sp.sin(theta),
         t * s_x - s_ox * (s_x**2 - 1) * (1 - sp.cos(theta)) - s_oy * (s_x * s_y * (1 - sp.cos(theta)) - s_z * sp.sin(theta)) - s_oz * (s_x * s_z * (1 - sp.cos(theta)) + s_y * sp.sin(theta))],
        [s_x * s_y * (1 - sp.cos(theta)) + s_z * sp.sin(theta),
         (s_y**2 - 1) * (1 - sp.cos(theta)) + 1,
         s_y * s_z * (1 - sp.cos(theta)) - s_x * sp.sin(theta),
         t * s_y - s_ox * (s_x * s_y * (1 - sp.cos(theta)) + s_z * sp.sin(theta)) - s_oy * (s_y**2 - 1) * (1 - sp.cos(theta)) - s_oz * (s_y * s_z * (1 - sp.cos(theta)) - s_x * sp.sin(theta))],
        [s_x * s_z * (1 - sp.cos(theta)) - s_y * sp.sin(theta),
         s_y * s_z * (1 - sp.cos(theta)) + s_x * sp.sin(theta),
         (s_z**2 - 1) * (1 - sp.cos(theta)) + 1,
         t * s_z - s_ox * (s_x * s_z * (1 - sp.cos(theta)) - s_y * sp.sin(theta)) - s_oy * (s_y * s_z * (1 - sp.cos(theta)) + s_x * sp.sin(theta)) - s_oz * (s_z**2 - 1) * (1 - sp.cos(theta))],
        [0, 0, 0, 1]
    ])
    return S

# Main code
# Parameter Definitions
sp.init_printing()
theta1, theta2, theta3, a1, a2, a3 = sp.symbols('theta1 theta2 theta3 a1 a2 a3', real=True)

# DH Table
a = sp.Matrix([a1, a2, a3])
alpha = sp.Matrix([0, 0, 0])
d = sp.Matrix([0, 0, 0])
T1 = DH(a[0], alpha[0], d[0], theta1)
T2 = DH(a[1], alpha[1], d[1], theta2)
T3 = DH(a[2], alpha[2], d[2], theta3)
T_Final = T1 * (T2 * T3)

# Forward Kinematics
print('Final End-Effector Position (DH method):')
P_DH = T_Final[:3, 3]
R_DH = T_Final[:3, :3]
display(P_DH)
print('=')
display(sp.simplify(P_DH))

# Screw-Based Analysis
S1 = SR(0, 0, 1, 0, 0, 0, theta1, 0)
S2 = SR(0, 0, 1, a1, 0, 0, theta2, 0)
S3 = SR(0, 0, 1, a1 + a2, 0, 0, theta3, 0)
Screw = S1 * (S2 * S3)

print('********************************************************************************')

# Screw-based Forward Kinematics
U0 = sp.Matrix([1, 0, 0])
V0 = sp.Matrix([0, 1, 0])
W0 = sp.Matrix([0, 0, 1])
E0 = sp.Matrix([a1 + a2 + a3, 0, 0])
S0 = sp.Matrix([[U0, V0, W0, E0], [0, 0, 0, 1]])
S_Final = Screw * S0
R_SR = S_Final[:3, :3]
P_SR = S_Final[:3, 3]
print('Final End-Effector Position (Screw method):')
display(P_SR)
print('=')
display(sp.simplify(P_SR))

print('********************************************************************************')

# Verification
print('The difference of End-Effector HT found by two methods:')
Error_T = sp.simplify(S_Final - T_Final)
display(sp.simplify(S_Final - T_Final))

"""## This code provides the forward kinematic solution of the SCARA robot with DH method:"""

import sympy as sp

# DH function
def DH(a, alpha, d, theta):
    # Construct the Denavit-Hartenberg (DH) transformation matrix
    T = sp.Matrix([[sp.cos(theta), -sp.sin(theta) * sp.cos(alpha), sp.sin(theta) * sp.sin(alpha), a * sp.cos(theta)],
                   [sp.sin(theta), sp.cos(theta) * sp.cos(alpha), -sp.cos(theta) * sp.sin(alpha), a * sp.sin(theta)],
                   [0, sp.sin(alpha), sp.cos(alpha), d],
                   [0, 0, 0, 1]])
    return T

# SR function
def SR(s_x, s_y, s_z, s_ox, s_oy, s_oz, theta, t):
    # Construct the Screw-based transformation matrix
    S = sp.Matrix([
        [(s_x**2 - 1) * (1 - sp.cos(theta)) + 1,
         s_x * s_y * (1 - sp.cos(theta)) - s_z * sp.sin(theta),
         s_x * s_z * (1 - sp.cos(theta)) + s_y * sp.sin(theta),
         t * s_x - s_ox * (s_x**2 - 1) * (1 - sp.cos(theta)) - s_oy * (s_x * s_y * (1 - sp.cos(theta)) - s_z * sp.sin(theta)) - s_oz * (s_x * s_z * (1 - sp.cos(theta)) + s_y * sp.sin(theta))],
        [s_x * s_y * (1 - sp.cos(theta)) + s_z * sp.sin(theta),
         (s_y**2 - 1) * (1 - sp.cos(theta)) + 1,
         s_y * s_z * (1 - sp.cos(theta)) - s_x * sp.sin(theta),
         t * s_y - s_ox * (s_x * s_y * (1 - sp.cos(theta)) + s_z * sp.sin(theta)) - s_oy * (s_y**2 - 1) * (1 - sp.cos(theta)) - s_oz * (s_y * s_z * (1 - sp.cos(theta)) - s_x * sp.sin(theta))],
        [s_x * s_z * (1 - sp.cos(theta)) - s_y * sp.sin(theta),
         s_y * s_z * (1 - sp.cos(theta)) + s_x * sp.sin(theta),
         (s_z**2 - 1) * (1 - sp.cos(theta)) + 1,
         t * s_z - s_ox * (s_x * s_z * (1 - sp.cos(theta)) - s_y * sp.sin(theta)) - s_oy * (s_y * s_z * (1 - sp.cos(theta)) + s_x * sp.sin(theta)) - s_oz * (s_z**2 - 1) * (1 - sp.cos(theta))],
        [0, 0, 0, 1]
    ])
    return S

# Main code
# Parameter Definitions
sp.init_printing()
theta1, theta2, theta4, d1, d3, d4, a1, a2 = sp.symbols('theta1 theta2 theta4 d1 d3 d4 a1 a2', real=True)

# DH Table
T1 = DH(a1, 0, d1, theta1)
T2 = DH(a2, np.pi, 0, theta2)
T3 = DH(0, 0, d3, 0)
T4 = DH(0, 0, d4, theta4)
T_Final = (T1 * T2) * (T4 * T3)

# Forward Kinematics
print('Final End-Effector Position (DH method):')
P_DH = T_Final[:3, 3]
R_DH = T_Final[:3, :3]

display(P_DH)
print('=')
display(sp.nsimplify(P_DH,tolerance=1e-10,rational=True))

print('********************************************************************************')

print('Final End-Effector Orientation (DH method):')
display(R_DH)
print('=')
display(sp.nsimplify(R_DH,tolerance=1e-10,rational=True))