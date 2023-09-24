%%
% An Introduction to Robotics,
% By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
% Copyright ARAS @2023
%
% This code provides the forward kinematic solution of the Stanford manipulator
% with DH and Screw methods

%% Parameter Definitions
clear; % Clear workspace
clc; % Clear command window

syms theta1 theta2 theta3 theta4 theta5 theta6 d2 d3 d6 real % Symbolic variables for joint angles and link lengths

%% DH Table:
% Problem 3.4 on the book: Table 3.4
% [T] = DH(a,alpha,d,theta) % Homogeneous Transformation
T1 = DH(0, -pi/2, 0, theta1); % Transformation matrix for joint 1
T1 = Matrix_Vpa(T1, 4, 4); % Convert to decimal approximation

T2 = DH(0, pi/2, d2, theta2); % Transformation matrix for joint 2
T2 = Matrix_Vpa(T2, 4, 4); % Convert to decimal approximation

T3 = DH(0, 0, d3, 0); % Transformation matrix for joint 3
T3 = Matrix_Vpa(T3, 4, 4); % Convert to decimal approximation

T4 = DH(0, -pi/2, 0, theta4); % Transformation matrix for joint 4
T4 = Matrix_Vpa(T4, 4, 4); % Convert to decimal approximation

T5 = DH(0, pi/2, 0, theta5); % Transformation matrix for joint 5
T5 = Matrix_Vpa(T5, 4, 4); % Convert to decimal approximation

T6 = DH(0, 0, d6, theta6); % Transformation matrix for joint 6
T6 = Matrix_Vpa(T6, 4, 4); % Convert to decimal approximation

% Calculate compound transformations
T12 = simplify(T1 * T2); % Transformation from joint 1 to joint 2
T34 = simplify(T3 * T4); % Transformation from joint 3 to joint 4
T56 = simplify(T5 * T6); % Transformation from joint 5 to joint 6

% Final Homogeneous Transformation
T_Final = simplify(T12 * T34 * T56);
T_Final = Matrix_Vpa(T_Final, 4, 4); % Convert to decimal approximation

% To visualize the final Homogeneous Transformation, type 'T_Final' in the command window.

disp('Final wrist Position (DH method):')
P_DH = T_Final(1:3, 4); % Separate the position vector

disp('Final wrist Orientation (DH method):')
R_DH = T_Final(1:3, 1:3); % Separate the rotation matrix of the End-Effector

%% Forward Kinematics (Screw-Based Analysis)
% Problem 3.12 on the book (Table 3.8 is used)
% [S] = SR(s_x,s_y,s_z,s_ox,s_oy,s_oz,theta,d)
S1 = SR(0, 0, 1, 0, 0, 0, theta1, 0); % Screw-based Homogeneous Transformation for joint 1
S2 = SR(1, 0, 0, 0, 0, 0, theta2, 0); % Screw-based Homogeneous Transformation for joint 2
S3 = SR(0, 1, 0, 0, 0, d2, 0, d3); % Screw-based Homogeneous Transformation for joint 3
S4 = SR(0, 1, 0, 0, 0, d2, theta4, 0); % Screw-based Homogeneous Transformation for joint 4
S5 = SR(0, 0, 1, 0, 0, d2, theta5, 0); % Screw-based Homogeneous Transformation for joint 5
S6 = SR(0, 1, 0, 0, 0, d2, theta6, 0); % Screw-based Homogeneous Transformation for joint 6

S12 = simplify(S1 * S2); % Compound Transformation for joints 1 and 2
S34 = simplify(S3 * S4); % Compound Transformation for joints 3 and 4
S56 = simplify(S5 * S6); % Compound Transformation for joints 5 and 6

Screw = simplify(S12 * simplify(S34 * S56)); % Compound Transformation for the entire manipulator

S13 = simplify(S12 * S3); % Compound Transformation for joints 1, 2, and 3

% To visualize the final Screw-based Homogeneous Transformation, type 'Screw' in the command window.

%% Screw-based Forward Kinematics
% position and orientation at initial state
U0 = [1; 0; 0];
V0 = [0; 0; -1];
W0 = [0; 1; 0];
P0 = [d2; 0; 0];
Ze = [0,0,0];
S0 = [U0, V0, W0, P0; Ze, 1];

% Screw-based Loop closure equation
S_Final = S13 * S0;
R_SR = S_Final(1:3, 1:3); % Orientation of end effector
P_SR = S_Final(1:3, 4); % Position of end effector

disp('Final Wrist Position (Screw method):')
P_SR = simplify(P_SR)

%% Verification
% Note that the initial position in the DH and Screw representation are different.
% In the DH method, the robot arm is in a vertical position to remain in the initial state,
% while in the above screw method, the arm is considered in a horizontal position at the initial state.
% This causes the two derivations to result in two different solutions.
% To remedy this, consider the notation given in Paul's book (page 56) and verify the solutions.

%% Paul's notation
% Screw-based Forward Kinematics
S1 = SR(0, 0, 1, 0, 0, 0, theta1, 0); % Screw-based Homogeneous Transformation for joint 1
S2 = SR(0, 1, 0, 0, 0, 0, theta2, 0); % Screw-based Homogeneous Transformation for joint 2
S3 = SR(0, 0, 1, 0, d2, 0, 0, d3); % Screw-based Homogeneous Transformation for joint 3
S4 = SR(0, 0, 1, 0, d2, 0, theta4, 0); % Screw-based Homogeneous Transformation for joint 4
S5 = SR(0, 1, 0, 0, d2, 0, theta5, 0); % Screw-based Homogeneous Transformation for joint 5
S6 = SR(0, 0, 1, 0, d2, 0, theta6,0); % Screw-based Homogeneous Transformation for joint 6

S12 = simplify(S1 * S2); % Compound Transformation for joints 1 and 2
S13 = simplify(S12 * S3); % Compound Transformation for joints 1, 2, and 3

% To visualize the final Screw-based Homogeneous Transformation, type 'Screw' in the command window.

% position and orientation at initial state
U0 = [1; 0; 0];
V0 = [0; 1; 0];
W0 = [0; 0; 1];
P0 = [0; d2; 0];
Ze = [0,0,0];
S0 = [U0, V0, W0, P0; Ze, 1];

% Screw-based Loop closure equation
S_Final = S13 * S0;
R_SR = S_Final(1:3, 1:3); % Orientation of end effector
P_SR = S_Final(1:3, 4); % Position of end effector

disp('Final Wrist Position (Screw method):')
P_SR = simplify(P_SR)

%% Verification
% To show that the two methods reach the same result, find the difference between their end-effector positions.
disp('The difference of Wrist position found by two methods:')
Error_T = simplify(P_SR - P_DH)