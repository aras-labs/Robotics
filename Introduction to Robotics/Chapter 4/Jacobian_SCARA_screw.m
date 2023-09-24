%% 
% 	An Introduction to Robotics, 
%   By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
%   Copyright ARAS @2023

%   This code provides the Screw-based Jacobian derivation of 
%   SCARA robot
clear all
syms('d2','d3','th1','th2','th4','th5','th6')
th3=pi/2;  % Set the value of th3 to pi/2

R01=[cos(th1) 0 -sin(th1)
    sin(th1) 0 cos(th1) 
    0 -1 0];
R12=[cos(th2) 0 sin(th2)
    sin(th2) 0 -cos(th2) 
    0 1 0];
R23=[0 1 0
    -1 0 0
    0 0 1];
R34=[cos(th4) 0 -sin(th4)
    sin(th4) 0 cos(th4) 
    0 -1 0];
R45=[cos(th5) 0 sin(th5)
    sin(th5) 0 -cos(th5) 
    0 1 0];
R56=[cos(th6) -sin(th6) 0
    sin(th6)  cos(th6) 0 
    0 0 1];
R02=simplify(R01*R12);  % Calculate the composite rotation matrix from frame 0 to frame 2
R03=simplify(R02*R23);  % Calculate the composite rotation matrix from frame 0 to frame 3
R35=simplify(R34*R45);  % Calculate the composite rotation matrix from frame 3 to frame 5
R36=simplify(R35*R56);  % Calculate the composite rotation matrix from frame 3 to frame 6

R10=simplify(inv(R01));  % Calculate the inverse of the rotation matrix from frame 1 to frame 0
R21=simplify(inv(R12));  % Calculate the inverse of the rotation matrix from frame 2 to frame 1
R32=inv(R23);  % Calculate the inverse of the rotation matrix from frame 3 to frame 2
R31=simplify(R32*R21);  % Calculate the composite rotation matrix from frame 2 to frame 1
R30=simplify(inv(R03));  % Calculate the inverse of the rotation matrix from frame 3 to frame 0

z=[0;0;1];  % Define the direction of the z-axis of the base frame (frame 0)
s4=[0;0;1]; so4=[0;0;0];  % Define the direction and axis of joint 4
s5=R34*z;  % Define the direction of the z-axis of frame 5
so5=so4+R34*[0;0;0];  % Define the axis of joint 5
s6=R35*z;  % Define the direction of the z-axis of frame 6
so6=so5+R35*[0;0;0];  % Define the axis of joint 6
s3=R32*z;  % Define the direction of the z-axis of frame 3
so3=so4-[0;0;d3];  % Define the axis of joint 3
s2=R31*z;  % Define the direction of the z-axis of frame 2
so2=so3-R32*[0;d2;0];  % Define the axis of joint 2
s1=R30*z;  % Define the direction of the z-axis of frame 1
so1=so2-R31*[0;0;0];  % Define the axis of joint 1

J1=[s1; cross(so1,s1)];  % Calculate the first column of the Jacobian matrix for joint 1
J2=[s2; cross(so2,s2)];  % Calculate the second column of the Jacobian matrix for joint 2
J3=[0;0;0; s3];  % Calculate the third column of the Jacobian matrix for joint 3
J4=[s4; cross(so4,s4)];  % Calculate the fourth column of the Jacobian matrix for joint 4
J5=[s5; cross(so5,s5)];  % Calculate the fifth column of the Jacobian matrix for joint 5
J6=[s6; cross(so6,s6)];  % Calculate the sixth column of the Jacobian matrix for joint 6

J=[J1 J2 J3 J4 J5 J6]  % Assemble the Jacobian matrix using the calculated columns
