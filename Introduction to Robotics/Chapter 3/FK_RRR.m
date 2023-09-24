%%
% An Introduction to Robotics,
% By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
% Copyright ARAS @2023
%
% This code provides the forward kinematic solution of the 3R robot
% with DH and Screw methods

%% Parameter Definitions
clear all; % Clear workspace
clc; % Clear command window

syms theta1 theta2 theta3 a1 a2 a3 real % Symbolic variables for joint angles and link lengths

%% DH Table:
% Problem 3.1 on the book (Table 3.1 is used)
a = [a1; a2; a3]; % Link lengths
al = [0; 0; 0]; % Link twists
d = [0; 0; 0]; % Link offsets

% DH(a,alpha,d,theta)
T1 = DH(a(1), al(1), d(1), theta1); % Transformation matrix for joint 1
T2 = DH(a(2), al(2), d(2), theta2); % Transformation matrix for joint 2
T3 = DH(a(3), al(3), d(3), theta3); % Transformation matrix for joint 3

% Calculate compound transformations
T_Final = T1 * simplify(T2 * T3); % Final Homogeneous Transformation

% Type "T_Final" on the command window to visualize
% the Final Homogeneous Transformation

%% Forward Kinematics
disp('Final End-Effector Position (DH method):')
P_DH = T_Final(1:3, 4); % Position of end effector
R_DH = T_Final(1:3, 1:3); % Orientation of end effector

%% Forward Kinematics (Screw-Based Analysis)
% Problem 3.9 on the book (Table 3.5 is used)
% [S] = SR(s_x, s_y, s_z, s_ox, s_oy, s_oz, theta, d)
S1 = SR(0, 0, 1, 0, 0, 0, theta1, 0); % Screw-based Homogeneous Transformation for joint 1
S2 = SR(0, 0, 1, a1, 0, 0, theta2, 0); % Screw-based Homogeneous Transformation for joint 2
S3 = SR(0, 0, 1, a1 + a2, 0, 0, theta3, 0); % Screw-based Homogeneous Transformation for joint 3

Screw = simplify(S1 * simplify(S2 * S3)); % Compound Transformation

% Final Compound Transformation
%
% Type "Screw" on the command window to visualize
% the Final Screw-based Homogeneous Transformation

%% Screw-based Forward Kinematics

% Position and orientation at initial state
U0 = [1; 0; 0];
V0 = [0; 1; 0];
W0 = [0; 0; 1];
E0 = [a1 + a2 + a3; 0; 0];
S0 = [U0, V0, W0, E0;
0, 0, 0, 1]; % Initial state HT

% Screw-based Loop closure equation
S_Final = Screw * S0;
R_SR = S_Final(1:3, 1:3); % Orientation of end effector
P_SR = S_Final(1:3, 4); % Position of end effector

disp('Final End-Effector Position (Screw method):')
P_SR = simplify(P_SR)

%% Verification
% To show the two methods reach the same result,
% see the difference of their homogeneous transformations
disp('The difference of End-Effector HT found by two methods:')
Error_T = simplify(S_Final - T_Final)