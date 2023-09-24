%% 
% 	An Introduction to Robotics, 
%   By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
%   Copyright ARAS @2023
%
%   This code provides the Jacobian derivation of the 3R robot  with
%   General and Screw methods By inspection

%% Parameter Definitions
clear; clc; clear all
syms th1 th2 th3 a1 a2 a3 real
a=[a1;a2;a3]; al=[0;0;0]; d=[0;0;0];

% Find Rotation Matrices by inspection
R01=[cos(th1) -sin(th1) 0
    sin(th1) cos(th1) 0
    0 0 1];
R12=[cos(th2) -sin(th2) 0
    sin(th2) cos(th2) 0
    0 0 1];
R23=[cos(th3) -sin(th3) 0
    sin(th3) cos(th3) 0
    0 0 1];
R02=simplify(R01*R12);  % Calculate the composite rotation matrix from frame 0 to frame 2
R03=simplify(R02*R23);  % Calculate the composite rotation matrix from frame 0 to frame 3
R10=simplify(inv(R01)); % Inverse of R01
R21=simplify(inv(R12)); % Inverse of R12
R20=simplify(inv(R02)); % Inverse of R02
R30=simplify(inv(R03)); % Inverse of R03

%% General Jacobian
% Find Jacobian column components by inspection 

% Find End-Efector components by inspection
z0=[0;0;1];      % Direction of the first axis of rotation 
z1=z0; z2=z0;

Pe2=R03*[a3;0;0];   % Calculate the position of the end-effector in frame 2 coordinates
Pe1=Pe2+R02*[a2;0;0];  % Calculate the position of the end-effector in frame 1 coordinates
Pe0=Pe1+R01*[a1;0;0];  % Calculate the position of the end-effector in frame 0 coordinates

% General Jacobian Eq 4.19 in the book
disp('First column of General Jacobian matrix:')
J1=[simplify(cross(z0,Pe0)); z0]  % Calculate the first column of the General Jacobian matrix
J2=[simplify(cross(z1,Pe1)); z1];  % Calculate the second column of the General Jacobian matrix
J3=[simplify(cross(z2,Pe2)); z2];  % Calculate the third column of the General Jacobian matrix

J=[J1 J2 J3];  % Assemble the General Jacobian matrix using the calculated columns
%   Type J on the command window to Visualize 
%   the Final General Jacobian Matrix of the robot

%%  Screw-based Jacobian
% Find Jacobian column components by inspection 
s1=[0;0;1]; s2=s1; s3=s1;
so3=R03*[-a3;0;0];  % Calculate the screw axis of the end-effector in frame 3 coordinates
so2=so3+R02*[-a2;0;0];  % Calculate the screw axis of the end-effector in frame 2 coordinates
so1=so2+R01*[-a1;0;0];  % Calculate the screw axis of the end-effector in frame 1 coordinates

disp('First column of Screw-based Jacobian matrix:')
JS1=[s1; simplify(cross(so1,s1))]  % Calculate the first column of the Screw-based Jacobian matrix
JS2=[s2; simplify(cross(so2,s2))];  % Calculate the second column of the Screw-based Jacobian matrix
JS3=[s3; simplify(cross(so3,s3))];  % Calculate the third column of the Screw-based Jacobian matrix

JS=[JS1 JS2 JS3];  % Assemble the Screw-based Jacobian matrix using the calculated columns
