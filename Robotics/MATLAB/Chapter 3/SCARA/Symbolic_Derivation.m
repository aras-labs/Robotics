%% 
%   Robotics Course, Professor: Prof. Hamid D. Taghirad
%   Aras.kntu.ac.ir/education/robotics
%   Copyright Ali.Hassani 2020
%
%   This code provides the various kinematic components of the SCARA robot
%%
clear;
clc;

syms theta1 theta2 theta4 d1 d3 d4 a1 a2
syms THETA THETA2 THETA3

%% DH Table
%[T] = DH(a,alpha,d,theta) %Input format
T1=DH(a1, 0, d1, theta1); T1=Matrix_Vpa(T1,4,4);
T2=DH(a2, pi, 0, theta2);  T2=Matrix_Vpa(T2,4,4);
T3=DH(0, 0, d3, 0);
T4=DH(0, 0, d4, theta4);  T4=Matrix_Vpa(T4,4,4);
T12=simplify(T1*T2);
T34=simplify(T4*T3);
T_Final=simplify(T12*T34);    T_Final=Matrix_Vpa(T_Final,4,4);
%% Forward Kinematics
P_DH=T_Final(1:3,4); 
R_DH=T_Final(1:3,1:3);
%% Forward Kinematics (Screw)
% [S] = SR(s_x,s_y,s_z,s_ox,s_oy,s_oz,theta,t)
S1=SR(0,0,1,0,0,0,theta1,0);
S2=SR(0,0,1,0,a1,0,theta2,0);
S3=SR(0,0,-1,0,a1+a2,0,0,d3);
S4=SR(0,0,-1,0,a1+a2,0,theta4,0);
S12=simplify(S1*S2);
S34=simplify(S3*S4);
Screw=simplify(S12*S34);

R0=[0  1  0
    1  0  0
    0  0  -1];
P0=[0;a1+a2;d1-d4];Ze=[0,0,0];
S0=[R0,P0;
    Ze,1];

S_Final=simplify(Screw*S0);
R_SR=S_Final(1:3,1:3);
P_SR=S_Final(1:3,4);P_SR=simplify(P_SR);
%% Verification
P_Position=simplify(P_DH-P_SR)
E_Rotation=simplify(R_DH-R_SR)
%% Final_Step: Export Forward Kinematics to a Functions!
