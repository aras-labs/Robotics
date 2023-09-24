%% 
% 	An Introduction to Robotics, 
%   By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
%   Copyright ARAS @2023
%
%   This code provides the Screw-based Jacobian derivation of 
%   the Stanford robot 
clear;
clc;
syms theta1 theta2 theta3 theta4 theta5 theta6 d2 d3 d6 real  % Declare symbolic variables for joint angles and distances

%% DH Table: 
% Problem 3.4 on the book: Table 3.4
% [T] = DH(a,alpha,d,theta)     % Homogeneous Transformation
% DH(a,alpha,d,theta)
a=[0;0;0;0;0;0];  % Define DH parameters a
al=[-pi/2;pi/2;0;-pi/2;pi/2;0];  % Define DH parameters alpha
d=[0;d2;d3;0;0;d6];  % Define DH parameters d

% Homogeneous Transformation
T01=DH(a(1),al(1),d(1),theta1); T01=Matrix_Vpa(T01,4,4);  % Calculate the transformation matrix from frame 0 to frame 1
R01=T01(1:3,1:3);  % Extract the rotation matrix from the transformation matrix

T12=DH(a(2),al(2),d(2),theta2); T12=Matrix_Vpa(T12,4,4);  % Calculate the transformation matrix from frame 1 to frame 2
R12=T12(1:3,1:3);  % Extract the rotation matrix from the transformation matrix

T23=DH(a(3),al(3),d(3),theta3); T23=Matrix_Vpa(T23,4,4);  % Calculate the transformation matrix from frame 2 to frame 3
R23=T23(1:3,1:3);  % Extract the rotation matrix from the transformation matrix

T34=DH(a(4),al(4),d(4),theta4); T34=Matrix_Vpa(T34,4,4);  % Calculate the transformation matrix from frame 3 to frame 4
R34=T34(1:3,1:3);  % Extract the rotation matrix from the transformation matrix

T45=DH(a(5),al(5),d(5),theta5); T45=Matrix_Vpa(T45,4,4);  % Calculate the transformation matrix from frame 4 to frame 5
R45=T45(1:3,1:3);  % Extract the rotation matrix from the transformation matrix

T56=DH(a(6),al(6),d(6),theta6); T56=Matrix_Vpa(T56,4,4);  % Calculate the transformation matrix from frame 5 to frame 6
R56=T56(1:3,1:3);  % Extract the rotation matrix from the transformation matrix

% Calculate compound transformations
T02=simplify(T01*T12);       R02=T02(1:3,1:3);  % Calculate the composite transformation from frame 0 to frame 2 and extract the rotation matrix
T03=simplify(T02*T23);       R03=T03(1:3,1:3);  % Calculate the composite transformation from frame 0 to frame 3 and extract the rotation matrix
T04=simplify(T03*T34);       R04=T04(1:3,1:3);  % Calculate the composite transformation from frame 0 to frame 4 and extract the rotation matrix
T05=simplify(T04*T45);       R05=T05(1:3,1:3);  % Calculate the composite transformation from frame 0 to frame 5 and extract the rotation matrix
T06=simplify(T05*T56);       R06=T06(1:3,1:3);  % Calculate the composite transformation from frame 0 to frame 6 and extract the rotation matrix

%% Jacobian Analysis
% Screw-based Jacobian: Iterative method
% Find End-Effector components by inspection
z=[0;0;1];      % Direction of the first axis of rotation 
so7=[0;0;0];    % Consider the axis of the final coordinate on the end-effector 

% Iterative method 
% Formulation 4.38 in the book
for i=1:6;
  p(:,i)=[a(i); d(i)*sin(al(i)); d(i)*cos(al(i))];  % Calculate the position vectors of each joint
end

% Formulation 4.37 in the book 
s6=R05*z;   so6=so7-R06*p(:,6);  % Calculate the screw axis and point on the axis for joint 6
s5=R04*z;   so5=so6-R05*p(:,5);  % Calculate the screw axis and point on the axis for joint 5
s4=R03*z;   so4=so5-R04*p(:,4);  % Calculate the screw axis and point on the axis for joint 4
s3=R02*z;   so3=so4-R03*p(:,3);  % Calculate the screw axis and point on the axis for joint 3
s2=R01*z;   so2=so3-R02*p(:,2);  % Calculate the screw axis and point on the axis for joint 2
s1=z;       so1=so2-R01*p(:,1);  % Calculate the screw axis and point on the axis for joint 1

J1=[s1; simplify(cross(so1,s1))];  % Calculate the first column of the Jacobian matrix for joint 1
J2=[s2; simplify(cross(so2,s2))];  % Calculate the second column of the Jacobian matrix for joint 2
J3=[0*s3; s3];                      % Prismatic joint, set to 0 since it has no rotational component
J4=[s4; simplify(cross(so4,s4))];  % Calculate the fourth column of the Jacobian matrix for joint 4
J5=[s5; simplify(cross(so5,s5))];  % Calculate the fifth column of the Jacobian matrix for joint 5

disp('Last column of Screw-based Jacobian matrix:')
J6=[s6; simplify(cross(so6,s6))]  % Calculate the sixth column of the Jacobian matrix for joint 6

J=[J1 J2 J3 J4 J5 J6];  % Assemble the Jacobian matrix using the calculated columns
%   Type JS on the command window to Visualize 
%   the Final Screw-based Jacobian Matrix of the robot
