%% 
% 	An Introduction to Robotics, 
%   By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
%   Copyright ARAS @2023
%
%   This code provides the various kinematic components of the 3R robot
%%
clear;
clc;

syms theta1 theta2 theta3 a1 a2 a3
syms THETA THETA2 THETA3

%% DH Table
a=[a1;a2;a3];
al=[0;0;0];
d=[0;0;0];
T1=DH(a(1),al(1),d(1),theta1); T1=Matrix_Vpa(T1,4,4);  
R1=T1(1:3,1:3);
T2=DH(a(2),al(2),d(2),theta2); T2=Matrix_Vpa(T2,4,4);  
R2=T2(1:3,1:3);
T3=DH(a(3),al(3),d(3),theta3); T3=Matrix_Vpa(T3,4,4);  
R3=T3(1:3,1:3);
T12=simplify(T1*T2);        R12=T12(1:3,1:3);
T13=simplify(T12*T3);       R13=T13(1:3,1:3);
T_Final=simplify(T13);
%% Forward Kinematics
P_DH=T_Final(1:3,4); 
R_DH=T_Final(1:3,1:3);
%% Forward Kinematics (Screw)
%[S] = SR(s__x,s__y,s__z,s__ox,s__oy,s__oz,theta,t)
S1=SR(0,0,1,0,0,0,theta1,0);       %R1=S1(1:3,1:3);
S2=SR(0,0,1,a2,0,0,theta2,0);      %R2=S2(1:3,1:3);
S3=SR(0,0,1,a3,0,0,theta3,0);      %R3=S3(1:3,1:3);
S12=simplify(S1*S2);               %R12=S12(1:3,1:3);
S13=simplify(S12*S3);              %R13=S13(1:3,1:3);
Screw=S1*simplify(S2*S3);

U0=[1;0;0];V0=[0;0;1];W0=[0;-1;0];P0=[a2+a3;0;0];Ze=[0,0,0];
S0=[U0,V0,W0,P0;Ze,1];

S_Final=Screw*S0;
R_SR=S_Final(1:3,1:3);
P_SR=S_Final(1:3,4);P_SR=simplify(P_SR);
%% Verification
P_Position=simplify(P_DH-P_SR)
E_Rotation=simplify(R_DH-R_SR)
%% Final_Step: Export Forward Kinematics to a Functions!

%
%% Jacobian Analysis
z=[0;0;1]; so4=[0;0;0]; 
for i=1:3;
  p(:,i)=[a(i); d(i)*sin(al(i)); d(i)*cos(al(i))];
end
s3=R12*z; so3=so4-R13*p(:,3);
s2=R1*z; so2=so3-R12*p(:,2);
s1=z;  so1=so2-R1*p(:,1);

J1=[s1; simplify(cross(so1,s1))];
J2=[s2; simplify(cross(so2,s2))];
J3=[s3; simplify(cross(so3,s3))];

J=[J1 J2 J3];
%J=J(3:5)