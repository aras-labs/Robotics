%% 
% 	An Introduction to Robotics, 
%   By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
%   Copyright ARAS @2023
%
%   This program solve Inverse kinematics and finds 
%   velocity and acceleration variablles of the 3R planar robot
%
function qs = IK (xs, SP)
x=xs(1:3); dx=xs(4:6); d2x=xs(7:9);

%   Parameters initialization
a1=SP.a(1); a2=SP.a(2); a3=SP.a(3);
xe=x(1);    ye=x(2);    the=x(3);

%   Inverse Kinematics
xp=xe-a3*cos(the);                      % Eq. 3.45
yp=ye-a3*sin(the);

c2=(xp^2+yp^2-a1^2-a2^2)/(2*a1*a2);     % Eq. 3.48
s2=-sqrt(1-c2^2);
th2=atan2(s2,c2);

b1=a1+a2*c2;                            %Eq. 3.51
b2=a2*s2;
th1=atan2(yp,xp)-atan2(b2,b1);          % Elbow up

th3= the-(th1+th2);

q=[th1; th2; th3];

%   Solve for the velocities
q1=q(1);q2=q(2);q3=q(3);
c1=cos(q1);s1=sin(q1);
c2=cos(q2);s2=sin(q2);
c3=cos(q3);s3=sin(q3);
c12=cos(q1+q2);s12=sin(q1+q2);
c23=cos(q2+q3);s23=sin(q2+q3);
c123=cos(q1+q2+q3);s123=sin(q1+q2+q3);

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
J=[J1 J2 J3]; Jinv = pinv(J); 
dq=Jinv*dx;

%   Solve for the accelerations
dq1=dq(1); dq2=dq(2); dq3=dq(3);
dq12=dq1+dq2; dq123=dq12+dq3;

dJ1=-[a1*c1*dq1+a2*c12*dq12+a3*c123*dq123;
      a1*s1*dq1+a2*s12*dq12+a3*s123*dq123;
      0];
dJ2=-[a2*c12*dq12+a3*c123*dq123;
      a2*s12*dq12+a3*s123*dq123;
      0];
dJ3=-[a3*c123*dq123;
      a3*s123*dq123;
      0];
dJ=[dJ1 dJ2 dJ3];
d2q=Jinv*(d2x-dJ*dq);

qs=[q; dq; d2q];

