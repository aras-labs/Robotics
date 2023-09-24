%% 
% 	An Introduction to Robotics, 
%   By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
%   Copyright ARAS @2023
%
%   This code provides the General Jacobian derivation of 
%   the SCARA manipulator in three different frames

%% Parameter Definitions
clear all, clc
syms('d1','d3','d4','a1','a2','th1','th2','th4')  % Symbolic variables for joint distances and angles
syms('p1','p2','p3','p4','pp1','pp2','pp3','pp4')  % Symbolic variables for intermediate positions
syms('J1','J2','J3','J4','J')  % Symbolic variables for Jacobian components
d2=0;a3=0;a4=0;th3=0;  % Initialize some parameters to zero (might not be used in this specific example)

% Rotation matrices
R1=[cos(th1) -sin(th1) 0
    sin(th1) cos(th1) 0
    0 0 1];
R2=[cos(th2) sin(th2) 0
    sin(th2) -cos(th2) 0
    0 0 -1];
R3=[cos(th4) -sin(th4) 0
    sin(th4) cos(th4) 0
    0 0 1];
R12=R1*R2;  % Calculate the composite rotation matrix from frame 1 to frame 2
R123=R12*R3;  % Calculate the composite rotation matrix from frame 1 to frame 3
R21=simplify(inv(R12));  % Calculate the inverse of the rotation matrix from frame 2 to frame 1

%%
%    General Jacobian matrix components
%
z0=[0;0;1]; z1=z0;  % Define the directions of the rotational axes of joints 1 and 2
z2=-z0; z3=z2;  % Define the direction of the rotational axis of joint 3 (opposite to z0)

p4=[0;0;-d4];  % Define the position vector of the end-effector in frame 4 coordinates
p3=R123*[0;0;d3]+p4;  % Calculate the position vector of the end-effector in frame 3 coordinates
p2=simplify(R12*[a2;0;0]+p3);  % Calculate the position vector of joint 2 in frame 2 coordinates
p1=simplify(R1*[a1;0;d1]+p2);  % Calculate the position vector of joint 1 in frame 1 coordinates

J3=cross(z2,p3);  % Calculate the screw axis of joint 3
J2=cross(z1,p2);  % Calculate the screw axis of joint 2
J1=cross(z0,p1);  % Calculate the screw axis of joint 1

disp('SCARA Jacobian matrix:')
J=[J1 J2 z2 J3  % Assemble the Jacobian matrix using the calculated screw axes
    z1 z2 0*z3 z3]
