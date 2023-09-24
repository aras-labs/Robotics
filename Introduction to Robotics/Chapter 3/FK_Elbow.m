%%
% An Introduction to Robotics,
% By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
% Copyright ARAS @2023
%
% This code provides the forward kinematic solution of the Elbow manipulator
% with DH and Screw methods

%% Parameter Definitions

clear; % Clear workspace
clc; % Clear command window

syms theta1 theta2 theta3 d1 a2 a3 real % Symbolic variables for joint angles and link lengths

%% DH Table
% Define the Denavit-Hartenberg (DH) parameters for the manipulator's joints.
% Problem 3.2 in the book is used to illustrate the DH table.(Table 3.2 is used)
% DH(a, alpha, d, theta)
T1 = DH(0, pi/2, d1, theta1); % Transformation matrix for joint 1
T2 = DH(a2, 0, 0, theta2); % Transformation matrix for joint 2
T3 = DH(a3, 0, 0, theta3); % Transformation matrix for joint 3

T1 = Matrix_Vpa(T1, 4, 4); % Convert to decimal approximation
T_Final = T1 * simplify(T2 * T3); % Final Homogeneous Transformation

% To visualize the final Homogeneous Transformation, type 'T_Final' in the command window.

%% Forward Kinematics
disp('Final End-Effector Position (DH method):')
P_DH = T_Final(1:3, 4); % Position of end effector
R_DH = T_Final(1:3, 1:3); % Orientation of end effector

%% Forward Kinematics (Screw-Based Analysis)
% Use Problem 3.10 in the book and Table 3.6 to define the screw parameters.
% [S] = SR(s_x, s_y, s_z, s_ox, s_oy, s_oz, theta, d)
S1 = SR(0, 0, 1, 0, 0, 0, theta1, 0); % Screw-based Homogeneous Transformation for joint 1
S2 = SR(0, -1, 0, 0, 0, d1, theta2, 0); % Screw-based Homogeneous Transformation for joint 2
S3 = SR(0, -1, 0, a2, 0, d1, theta3, 0); % Screw-based Homogeneous Transformation for joint 3

Screw = simplify(S1 * simplify(S2 * S3)); % Compound Transformation

% To visualize the final Screw-based Homogeneous Transformation, type 'Screw' in the command window.

%% Screw-based Forward Kinematics
% Define the initial state and perform the screw-based forward kinematics.
% Position and orientation at initial state
U0 = [1; 0; 0];
V0 = [0; 0; 1];
W0 = [0; -1; 0];
P0 = [a2 + a3; 0; d1];
S0 = [U0, V0, W0, P0;
0, 0, 0, 1]; % Initial state HT

% Solve the screw-based loop closure equation.
S_Final = Screw * S0;
R_SR = S_Final(1:3, 1:3); % Orientation of end effector
P_SR = S_Final(1:3, 4); % Position of end effector

disp('Final End-Effector Position (Screw method):')
P_SR = simplify(P_SR)

%% Verification
% Compare the results obtained from the two methods to check for consistency.
% Find the difference between the end-effector Homogeneous Transformations obtained by the two methods.
disp('The difference of End-Effector HT found by two methods:')
Error_T = simplify(S_Final - T_Final)