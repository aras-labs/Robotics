%% 
%   Robotics Course, Professor: Prof. Hamid D. Taghirad
%   Aras.kntu.ac.ir/education/robotics
%   Copyright Ali.Hassani 2020
%
%   This code provides the kinematic details of the 6DOF Elbow manipulator
%%
clear;
clc;

syms theta1 theta2 theta3 theta4 theta5 theta6 a2 a3 a4 d6 

%% DH Table
% DH(a,alpha,d,theta)
a=[0;a2;a3;a4;0;0];
al=[pi/2;0;0;-pi/2;pi/2;0];
d=[0;0;0;0;0;d6];
T1=DH(a(1),al(1),d(1),theta1); T1=Matrix_Vpa(T1,4,4);  
R1=T1(1:3,1:3);
T2=DH(a(2),al(2),d(2),theta2); T2=Matrix_Vpa(T2,4,4);  
R2=T2(1:3,1:3);
T3=DH(a(3),al(3),d(3),theta3); T3=Matrix_Vpa(T3,4,4);  
R3=T3(1:3,1:3);
T4=DH(a(4),al(4),d(4),theta4); T4=Matrix_Vpa(T4,4,4);  
R4=T4(1:3,1:3);
T5=DH(a(5),al(5),d(5),theta5); T5=Matrix_Vpa(T5,4,4);  
R5=T5(1:3,1:3);
T6=DH(a(6),al(6),d(6),theta6); T6=Matrix_Vpa(T6,4,4);  
R6=T6(1:3,1:3);
T12=simplify(T1*T2);        R12=T12(1:3,1:3);
T13=simplify(T12*T3);       R13=T13(1:3,1:3);
T14=simplify(T13*T4);       R14=T14(1:3,1:3);
T15=simplify(T14*T5);       R15=T15(1:3,1:3);
T16=simplify(T15*T6);       R16=T16(1:3,1:3);

P_DH=T16(1:3,4); 
R_DH=T16(1:3,1:3);
%% Forward Kinematics (Screw)
%[S] = SR(s__x,s__y,s__z,s__ox,s__oy,s__oz,theta,d)
S1=SR(0,0,1,0,0,0,theta1,0);        %R1=S1(1:3,1:3);
S2=SR(0,-1,0,0,0,0,theta2,0);       %R2=S2(1:3,1:3);
S3=SR(0,-1,0,a2,0,0,theta3,0);      %R3=S3(1:3,1:3);
S4=SR(0,-1,0,a2+a3,0,0,theta4,0);   %R4=S4(1:3,1:3);
S5=SR(0,0, 1,a2+a3+a4,0,0,theta5,0);%R5=S5(1:3,1:3);
S6=SR(1,0,0,0,0,0,theta6,0);        %R6=S6(1:3,1:3);
S12=simplify(S1*S2);                %R12=S12(1:3,1:3);
S13=simplify(S12*S3);               %R13=S13(1:3,1:3);
S14=simplify(S13*S4);               %R14=S14(1:3,1:3);
S15=simplify(S14*S5);               %R15=S15(1:3,1:3);
S16=simplify(S15*S6);               %R16=S16(1:3,1:3);
Screw=simplify(S16);                
% 
% 
U0=[0;0;1];V0=[0;-1;0];W0=[1;0;0];P0=[a2+a3+a4+d6;0;0];Ze=[0,0,0];
S0=[U0,V0,W0,P0;Ze,1];
% 
S_Final=Screw*S0;
R_SR=S_Final(1:3,1:3);
P_SR=S_Final(1:3,4);P_SR=simplify(P_SR);

%
%% Jacobian Analysis
z=[0;0;1]; so7=[0;0;0]; 
for i=1:6;
  p(:,i)=[a(i); d(i)*sin(al(i)); d(i)*cos(al(i))];
end

s6=R15*z; so6=so7-R16*p(:,6);
s5=R14*z; so5=so6-R15*p(:,5);
s4=R13*z; so4=so5-R14*p(:,4);
s3=R12*z; so3=so4-R13*p(:,3);
s2=R1*z; so2=so3-R12*p(:,2);
s1=z;  so1=so2-R1*p(:,1);

J1=[s1; simplify(cross(so1,s1))];
J2=[s2; simplify(cross(so2,s2))];
J3=[s3; simplify(cross(so3,s3))];
J4=[s4; simplify(cross(so4,s4))];
J5=[s5; simplify(cross(so5,s5))];
J6=[s6; simplify(cross(so6,s6))];

J=[J1 J2 J3 J4 J5 J6];

% Jacobian with reference to frame{4}

R44=eye(3,3);
R43=[cos(theta4) sin(theta4) 0;
    0 0 -1; -sin(theta4) cos(theta4) 0];
R32=[cos(theta3)  sin(theta3) 0;
    -sin(theta3)  cos(theta3) 0;
    0   0   1];
R21=[cos(theta2)  sin(theta2) 0;
    -sin(theta2)  cos(theta2) 0;
    0   0   1];
R10=[cos(theta1) sin(theta1) 0;
    0 0 1; -sin(theta1) cos(theta1) 0];
R42=simplify(R43*R32);
R41=simplify(R42*R21);
R40=simplify(R41*R10);

so5=[0;0;0]; 
s4=R43*z; so4=so5-R44*p(:,4);
s3=R42*z; so3=so4-R43*p(:,3);
s2=R41*z; so2=so3-R42*p(:,2);
s1=R40*z; so1=so2-R41*p(:,1);

J1=[s1; simplify(cross(so1,s1))];
J2=[s2; simplify(cross(so2,s2))];
J3=[s3; simplify(cross(so3,s3))];
J4=[s4; simplify(cross(so4,s4))];

J=[J1 J2 J3 J4];

% Jacobian with reference to frame{3}

R31=simplify(R32*R21);
R30=simplify(R31*R10);

so4=[0;0;0]; 
s3=R32*z; so3=so4-p(:,3);
s2=R31*z; so2=so3-R32*p(:,2);
s1=R30*z; so1=so2-R31*p(:,1);

J1=[s1; simplify(cross(so1,s1))];
J2=[s2; simplify(cross(so2,s2))];
J3=[s3; simplify(cross(so3,s3))];
J=[J1 J2 J3];
