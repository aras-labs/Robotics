%% 
% 	An Introduction to Robotics, 
%   By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
%   Copyright ARAS @2023
%
%   This function generates the Christoffel Matrix of the 3R Robot
%   at configuration q
%
function [C] = C_Matrix(q,dq)

%% Load Physical Parameters of Robot
% Define Numeric Variables
run("Parameters.m");
a1=a(1);a2=a(2);a3=a(3);
m1=m(1);m2=m(2);m3=m(3);
q1=q(1);q2=q(2);q3=q(3);
dq1=dq(1);dq2=dq(2);dq3=dq(3);
c1=cos(q1);s1=sin(q1);
c2=cos(q2);s2=sin(q2);
c3=cos(q3);s3=sin(q3);
c12=cos(q1+q2);s12=sin(q1+q2);
c23=cos(q2+q3);s23=sin(q2+q3);
c123=cos(q1+q2+q3);s123=sin(q1+q2+q3);

% C Matric components
C11=0;
C12=-a1*(a2*s2*m2+(2*a2*s2+a3*s23)*m3)*(dq1+dq2/2)-a1/2*a3*s23*m3*dq3;
C13=-a3*(a1*s23+a2+s3)*m3*(dq1+dq2/2)-a3/2*(a1*s23+2*a2*s3)*m3*dq2;
C21=a1/2*(a2*s2*(m2+2*m3)+a3*s23*m3)*(dq1+dq2/2)+a1/4*a3*s23*m3*dq3;
C22=-a1/4*(a2*m2*s2+2*a2*m3*s2+a3*m3*s23)*dq1;
C23=-a3*m3*((a2*s3+a1/4*s23)*dq1+a2*s3*dq2+a2/2*s3*dq3);
C31=a3*m3*((a1*s23+a2*s3)/2*dq1+(a1*s23+2*a1*s23)/4*dq2+...
    (a1*s23+a3*s3)/4*dq3);
C32=a3/2*m3*((a2*s3-a1/2*s23)*dq1+a2*s3*dq2+a2/2*s3*dq3);
C33=-a3/4*m3*((a1*s23+a2*s3)*dq1+a2*s3*dq2);

% C Matrix
C=[C11 C12 C13;
    C21 C22 C23;
    C31 C32 C33];

end

