%%
% An Introduction to Robotics,
% By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
% Copyright ARAS @2023
%
% This code provides the forward kinematic solution of the SCARA robot
% with DH method

%% Parameter Definitions

clear; % Clear workspace
clc; % Clear command window

syms theta1 theta2 theta4 d1 d3 d4 a1 a2 real % Symbolic variables for joint angles and link lengths

%% DH Table
% Problem 3.3 on the book(Table 3.3 is used)
% DH(a,alpha,d,theta)
T1 = DH(a1, 0, d1, theta1); % Transformation matrix for joint 1
T1 = Matrix_Vpa(T1, 4, 4); % Convert to decimal approximation

T2 = DH(a2, pi, 0, theta2); % Transformation matrix for joint 2
T2 = Matrix_Vpa(T2, 4, 4); % Convert to decimal approximation

T3 = DH(0, 0, d3, 0); % Transformation matrix for joint 3

T4 = DH(0, 0, d4, theta4); % Transformation matrix for joint 4
T4 = Matrix_Vpa(T4, 4, 4); % Convert to decimal approximation

% Calculate compound transformations
T12 = simplify(T1 * T2); % Transformation from joint 1 to joint 2
T34 = simplify(T4 * T3); % Transformation from joint 3 to joint 4

T_Final = simplify(T12 * T34); % Final Homogeneous Transformation

%% Forward Kinematics
disp('Final End-Effector Position (DH method):')
P_DH = T_Final(1:3, 4); % Position of end effector
disp('Final End-Effector Orientation (DH method):')
R_DH = T_Final(1:3, 1:3); % Orientation of end effector