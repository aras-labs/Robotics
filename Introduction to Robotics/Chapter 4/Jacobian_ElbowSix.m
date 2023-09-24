%% Introduction and Copyright

% An Introduction to Robotics,
% By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
% Copyright ARAS @2023

%% Code Description

% This MATLAB code calculates the Screw-based Jacobian of a 6-degree-of-freedom (DOF) Elbow manipulator in three different frames.

%% Parameter Definitions

% Clear the workspace and the command window
clear; clc;

% Symbolic variables
syms theta1 theta2 theta3 theta4 theta5 theta6 a2 a3 a4 d6 real

%% DH Table

% Define the Denavit-Hartenberg (DH) parameters for the manipulator
% based on Problem 4.7, Table 4.1
% DH(a, alpha, d, theta)
a = [0; a2; a3; a4; 0; 0];
alpha = [pi/2; 0; 0; -pi/2; pi/2; 0];
d = [0; 0; 0; 0; 0; d6];

% Homogeneous Transformations
T01 = DH(a(1), alpha(1), d(1), theta1);   % Calculate the transformation matrix from frame 0 to frame 1
T01 = Matrix_Vpa(T01, 4, 4);              % Convert the matrix elements to high precision
R01 = T01(1:3, 1:3);                      % Extract the rotation matrix from the homogeneous transformation

T12 = DH(a(2), alpha(2), d(2), theta2);   % Calculate the transformation matrix from frame 1 to frame 2
T12 = Matrix_Vpa(T12, 4, 4);
R12 = T12(1:3, 1:3);

T23 = DH(a(3), alpha(3), d(3), theta3);   % Calculate the transformation matrix from frame 2 to frame 3
T23 = Matrix_Vpa(T23, 4, 4);
R23 = T23(1:3, 1:3);

T34 = DH(a(4), alpha(4), d(4), theta4);   % Calculate the transformation matrix from frame 3 to frame 4
T34 = Matrix_Vpa(T34, 4, 4);
R34 = T34(1:3, 1:3);

T45 = DH(a(5), alpha(5), d(5), theta5);   % Calculate the transformation matrix from frame 4 to frame 5
T45 = Matrix_Vpa(T45, 4, 4);
R45 = T45(1:3, 1:3);

T56 = DH(a(6), alpha(6), d(6), theta6);   % Calculate the transformation matrix from frame 5 to frame 6
T56 = Matrix_Vpa(T56, 4, 4);
R56 = T56(1:3, 1:3);

% Calculate compound transformations
T02 = simplify(T01 * T12);   % Calculate the compound transformation from frame 0 to frame 2
R02 = T02(1:3, 1:3);         % Extract the rotation matrix from the compound transformation

T03 = simplify(T02 * T23);   % Calculate the compound transformation from frame 0 to frame 3
R03 = T03(1:3, 1:3);

T04 = simplify(T03 * T34);   % Calculate the compound transformation from frame 0 to frame 4
R04 = T04(1:3, 1:3);

T05 = simplify(T04 * T45);   % Calculate the compound transformation from frame 0 to frame 5
R05 = T05(1:3, 1:3);

T06 = simplify(T05 * T56);   % Calculate the compound transformation from frame 0 to frame 6
R06 = T06(1:3, 1:3);

%% Find the inverse rotation matrices by their transpose

R44 = eye(3, 3);            % Initialize the inverse rotation matrix for frame 4 as an identity matrix
R43 = R34';                 % Calculate the inverse rotation matrix for frame 3 by taking the transpose of R34
R32 = R23';                 % Calculate the inverse rotation matrix for frame 2 by taking the transpose of R23
R21 = R12';                 % Calculate the inverse rotation matrix for frame 1 by taking the transpose of R12
R10 = R01';                 % Calculate the inverse rotation matrix for frame 0 by taking the transpose of R01

% Calculate compound transformations of inverse matrices
R42 = simplify(R43 * R32);  % Calculate the compound transformation from frame 4 to frame 2 using the inverse matrices
R41 = simplify(R42 * R21);  % Calculate the compound transformation from frame 4 to frame 1 using the inverse matrices
R40 = simplify(R41 * R10);  % Calculate the compound transformation from frame 4 to frame 0 using the inverse matrices
R31 = simplify(R32 * R21);  % Calculate the compound transformation from frame 3 to frame 1 using the inverse matrices
R30 = simplify(R31 * R10);  % Calculate the compound transformation from frame 3 to frame 0 using the inverse matrices

%% Jacobian Analysis

% Problem 4.7
% Screw-based Jacobian: Iterative method
% Find End-Effector components by inspection

z = [0; 0; 1]; % Direction of the first axis of rotation
so7 = [0; 0; 0]; % Consider the axis of the final coordinate on the end-effector

% Iterative method
% Formulation 4.38 in the book

% Calculate the position vectors
for i = 1:6
    p(:, i) = [a(i); d(i) * sin(alpha(i)); d(i) * cos(alpha(i))];
end

% Formulation 4.37 in the book
s6 = R05 * z;               % Calculate the screw axis for joint 6 in frame 0 coordinates
so6 = so7 - R06 * p(:, 6);  % Calculate the twist of joint 6 in frame 0 coordinates

s5 = R04 * z;               % Calculate the screw axis for joint 5 in frame 0 coordinates
so5 = so6 - R05 * p(:, 5);  % Calculate the twist of joint 5 in frame 0 coordinates

s4 = R03 * z;               % Calculate the screw axis for joint 4 in frame 0 coordinates
so4 = so5 - R04 * p(:, 4);  % Calculate the twist of joint 4 in frame 0 coordinates

s3 = R02 * z;               % Calculate the screw axis for joint 3 in frame 0 coordinates
so3 = so4 - R03 * p(:, 3);  % Calculate the twist of joint 3 in frame 0 coordinates

s2 = R01 * z;               % Calculate the screw axis for joint 2 in frame 0 coordinates
so2 = so3 - R02 * p(:, 2);  % Calculate the twist of joint 2 in frame 0 coordinates

s1 = z;                     % Calculate the screw axis for joint 1 in frame 0 coordinates
so1 = so2 - R01 * p(:, 1);  % Calculate the twist of joint 1 in frame 0 coordinates

J1 = [s1; simplify(cross(so1, s1))]; % Calculate the Jacobian column for joint 1
J2 = [s2; simplify(cross(so2, s2))]; % Calculate the Jacobian column for joint 2
J3 = [s3; simplify(cross(so3, s3))]; % Calculate the Jacobian column for joint 3
J4 = [s4; simplify(cross(so4, s4))]; % Calculate the Jacobian column for joint 4
J5 = [s5; simplify(cross(so5, s5))]; % Calculate the Jacobian column for joint 5

disp('Last column of Screw-based Jacobian matrix:')
J6 = [s6; simplify(cross(so6, s6))]; % Calculate the Jacobian column for joint 6

J = [J1 J2 J3 J4 J5 J6];    % Assemble the Screw-based Jacobian Matrix for the robot

% Type JS on the command window to visualize
% the Final Screw-based Jacobian Matrix of the robot

%% Jacobian Matrix with Reference to Frame {4}

so5 = [0; 0; 0];

s4 = R43 * z;
so4 = so5 - R44 * p(:, 4);

s3 = R42 * z;
so3 = so4 - R43 * p(:, 3);

s2 = R41 * z;
so2 = so3 - R42 * p(:, 2);

s1 = R40 * z;
so1 = so2 - R41 * p(:, 1);

disp('First column of Screw-based Jacobian matrix w.r.t frame Four:')
JF1 = [s1; simplify(cross(so1, s1))];
JF2 = [s2; simplify(cross(so2, s2))];
JF3 = [s3; simplify(cross(so3, s3))];
JF4 = [s4; simplify(cross(so4, s4))];

JF = [JF1 JF2 JF3 JF4];

% Type JF on the command window to visualize
% the Final Screw-based Jacobian Matrix of the robot
% with respect to frame four

%% Jacobian Matrix with Reference to Frame {3}

so4 = [0; 0; 0];

s3 = R32 * z;
so3 = so4 - p(:, 3);

s2 = R31 * z;
so2 = so3 - R32 * p(:, 2);

s1 = R30 * z;
so1 = so2 - R31 * p(:, 1);

disp('First column of Screw-based Jacobian matrix w.r.t frame Three:')
JT1 = [s1; simplify(cross(so1, s1))];
JT2 = [s2; simplify(cross(so2, s2))];
JT3 = [s3; simplify(cross(so3, s3))];
JT = [JT1 JT2 JT3];

% Type JT on the command window to visualize
% the Final Screw-based Jacobian Matrix of the robot
% with respect to frame three
