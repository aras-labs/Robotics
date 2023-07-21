%% 
%   Robotics Book, Professor: Prof. Hamid D. Taghirad
%   aras.kntu.ac.ir/education/robotics
%   Copyright 2022
%
%   This code provides the kinematic details of the Fanuc S-900W robot
%%
clear;
clc;

syms theta1 theta2 theta3 theta4 theta5 theta6 a1 a2 a3 d4 d6 
syms THETA THETA2 THETA3

%% DH Table
% [T] = DH(a,alpha,d,theta)
T1=DH(a1,pi/2,0,theta1);    T1=Matrix_Vpa(T1,4,4);
T2=DH(a2,0,0,theta2);       T2=Matrix_Vpa(T2,4,4);
T3=DH(a3,pi/2,0,theta3);    T3=Matrix_Vpa(T3,4,4);
T4=DH(0,-pi/2,d4,theta4);   T4=Matrix_Vpa(T4,4,4);
T5=DH(0,pi/2,0,theta5);     T5=Matrix_Vpa(T5,4,4);
T6=DH(0,0,d6,theta6);       T6=Matrix_Vpa(T6,4,4);
T12=simplify(T1*T2);
T13=simplify(T12*T3);
T45=simplify(T4*T5);
T46=simplify(T45*T6);
T_Final=simplify(T13*T46);  T_Final=Matrix_Vpa(T_Final,4,4);
%% Forward Kinematics
P_DH=T_Final(1:3,4); 
R_DH=T_Final(1:3,1:3);
%% Forward Kinematics (Screw)
% S1=SR(0,0,1,0,0,0,theta1,0);
% S2=SR(1,0,0,0,0,0,theta2,0);
% S3=SR(0,1,0,g,0,0,0,d3);
% S4=SR(0,1,0,g,0,0,theta4,0);
% S5=SR(0,0,1,g,0,0,theta5,0);
% S6=SR(0,1,0,g,0,0,theta6,0);
% S12=simplify(S1*S2);
% S34=simplify(S3*S4);
% S56=simplify(S5*S6);
% Screw=simplify(S12*simplify(S34*S56));
% 
% U0=[1;0;0];V0=[0;0;-1];W0=[0;1;0];P0=[g;h;0];Ze=[0,0,0];
% S0=[U0,V0,W0,P0;Ze,1];
% 
% S_Final=Screw*S0;
% R_SR=S_Final(1:3,1:3);
% P_SR=S_Final(1:3,4);P_SR=simplify(P_SR);
% 
