    %% Introduction and Copyright

% An Introduction to Robotics,
% By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
% Copyright ARAS @2023

%% Code Description

% This code provides the Jacobian derivation of the 3R robot
% with General and Screw methods

%% Parameter Definitions

% Clear workspace and command window
clear; clc;

% Symbolic variables
syms theta1 theta2 theta3 a1 a2 a3 real

%% DH Parameters

% Define the DH parameters
% Problem 3.1 in the book
a = [a1; a2; a3];
alpha = [0; 0; 0];
d = [0; 0; 0];

% Homogeneous Transformations
T1 = DH(a(1), alpha(1), d(1), theta1);  % Transformation matrix from frame 0 to frame 1
T1 = Matrix_Vpa(T1, 4, 4);  % Ensure numerical stability using Matrix_Vpa
R1 = T1(1:3, 1:3);  % Extract the rotation matrix from the transformation matrix

T2 = DH(a(2), alpha(2), d(2), theta2);  % Transformation matrix from frame 1 to frame 2
T2 = Matrix_Vpa(T2, 4, 4);
R2 = T2(1:3, 1:3);

T3 = DH(a(3), alpha(3), d(3), theta3);  % Transformation matrix from frame 2 to frame 3
T3 = Matrix_Vpa(T3, 4, 4);
R3 = T3(1:3, 1:3);

% Compound transformations
T12 = simplify(T1 * T2);  % Transformation matrix from frame 1 to frame 2
R12 = T12(1:3, 1:3);

T13 = simplify(T12 * T3);  % Transformation matrix from frame 1 to frame 3
R13 = T13(1:3, 1:3);

%% Jacobian Analysis
% General Jacobian: Iterative method
% Find End-Effector components by inspection

z0 = [0; 0; 1];      % Direction of the first axis of rotation
Pe3 = [0; 0; 0];    % Consider the axis of the final coordinate on the end-effector

% Iterative method
% Formulation 4.24 in the book

% Calculate the position vectors
for i = 1:3
    p(:, i) = [a(i) * cos(alpha(i)); a(i) * sin(alpha(i)); d(i)];
end

% Formulation 4.23 in the book
z1 = R1 * z0;
z2 = R12 * z0;
z3 = R13 * z0;
Pe2 = Pe3 + R13 * p(:, 3);
Pe1 = Pe2 + R12 * p(:, 2);
Pe0 = Pe1 + R1 * p(:, 1);

% General Jacobian Eq 4.19 in the book
disp('First column of General Jacobian matrix:')
J1 = [simplify(cross(z0, Pe0)); z0];
J2 = [simplify(cross(z1, Pe1)); z1];
J3 = [simplify(cross(z2, Pe2)); z2];

J = [J1 J2 J3];

% Type J on the command window to visualize
% the Final General Jacobian Matrix of the robot

%% Jacobian Analysis
% Screw-based Jacobian: Iterative method
% Find End-Effector components by inspection

z = [0; 0; 1];      % Direction of the first axis of rotation
so4 = [0; 0; 0];    % Consider the axis of the final coordinate on the end-effector

% Iterative method
% Formulation 4.38 in the book

% Calculate the position vectors
for i = 1:3
    p(:, i) = [a(i); d(i) * sin(alpha(i)); d(i) * cos(alpha(i))];
end

% Formulation 4.37 in the book
s3 = R12 * z;
so3 = so4 - R13 * p(:, 3);
s2 = R1 * z;
so2 = so3 - R12 * p(:, 2);
s1 = z;
so1 = so2 - R1 * p(:, 1);

% Screw-based Jacobian Eq 4.33 in the book
disp('First column of Screw-based Jacobian matrix:')
JS1 = [s1; simplify(cross(so1, s1))];
JS2 = [s2; simplify(cross(so2, s2))];
JS3 = [s3; simplify(cross(so3, s3))];

JS = [JS1 JS2 JS3];

% Type JS on the command window to visualize
% the Final Screw-based Jacobian Matrix of the robot
