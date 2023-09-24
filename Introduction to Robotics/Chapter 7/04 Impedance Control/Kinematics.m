%% 
% 	An Introduction to Robotics, 
%   By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
%   Copyright ARAS @2023
%
%   This program solve forward kinematics and finds Jacobian matrix 
%   of the 3R planar robot
%
function [Kin] = Kinematics(qs, SP)

% Joint variables
q  = qs(1:3); dq = qs(4:6);

%   Parameters initialization
a1=SP.a(1);a2=SP.a(2);a3=SP.a(3);
q1=q(1);q2=q(2);q3=q(3);
c1=cos(q1);s1=sin(q1);
c2=cos(q2);s2=sin(q2);
c3=cos(q3);s3=sin(q3);
c12=cos(q1+q2);s12=sin(q1+q2);
c23=cos(q2+q3);s23=sin(q2+q3);
c123=cos(q1+q2+q3);s123=sin(q1+q2+q3);

%   Forward Kinematics
Kin.x3 = a1*c1 + a2*c12 + a3*c123;
Kin.y3 = a1*s1 + a2*s12 + a3*s123;
Kin.p3 = q1 + q2 + q3;
Kin.x2 = a1*c1 + a2*c12;
Kin.y2 = a1*s1 + a2*s12;
Kin.p2 = q1 + q2;
Kin.x1 = a1*c1; 
Kin.y1=a1*s1;
Kin.p1 = q1;
Kin.x0 = 0; 
Kin.y0=0;
Kin.xrobot = [Kin.x0; Kin.x1; Kin.x2; Kin.x3]; 
Kin.yrobot=[Kin.y0; Kin.y1; Kin.y2; Kin.y3];

% Jacobian Matrix
J1 = [-(a1*s1 + a2*s12 + a3*s123);
        a1*c1 + a2*c12 + a3*c123;
        1];
J2 = [-(a2*s12 + a3*s123);
        a2*c12 + a3*c123;
        1];
J3 = [-a3*s123;
        a3*c123;
        1];
Kin.J = [J1  J2  J3];

% End Effector Velocities
dx = Kin.J * dq;
Kin.dx = dx(1);
Kin.dy = dx(2); 
Kin.dp = dx(3);

%%
%   perterbed  forward kinematics
%   Parameters initialization
a1=SP.a(1)*SP.pert;a2=SP.a(2)*SP.pert;a3=SP.a(3)*SP.pert;

%   Forward Kinematics
Kin.x3h = a1*c1 + a2*c12 + a3*c123;
Kin.y3h = a1*s1 + a2*s12 + a3*s123;
Kin.p3h = q1 + q2 + q3;
Kin.x2h = a1*c1 + a2*c12;
Kin.y2h = a1*s1 + a2*s12;
Kin.p2h = q1 + q2;
Kin.x1h = a1*c1; 
Kin.y1h = a1*s1;
Kin.p1h = q1;
Kin.x0h = 0; 
Kin.y0h=0;
Kin.xroboth = [Kin.x0h; Kin.x1h; Kin.x2h; Kin.x3h]; 
Kin.yroboth = [Kin.y0h; Kin.y1h; Kin.y2h; Kin.y3h];
% Jacobian Matrix
J1 = [-(a1*s1 + a2*s12 + a3*s123);
        a1*c1 + a2*c12 + a3*c123;
        1];
J2 = [-(a2*s12 + a3*s123);
        a2*c12 + a3*c123;
        1];
J3 = [-a3*s123;
        a3*c123;
        1];
Kin.Jh = [J1  J2  J3];

% End Effector Velocities
dxh = Kin.Jh * dq;
Kin.dxh = dxh(1);
Kin.dyh = dxh(2); 
Kin.dph = dxh(3);