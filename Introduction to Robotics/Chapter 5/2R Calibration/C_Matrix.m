%% 
% 	An Introduction to Robotics, 
%   By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
%   Copyright ARAS @2023
%
%   This function generates the Christoffel Matrix of the 2R Robot
%   at configuration q

function [C] = C_Matrix(q,dq)

%% Load Physical Parameters of Robot
% Define Numeric Variables
run("Parameters.m");
a1=a(1);a2=a(2);
m1=m(1);m2=m(2);
q1=q(1);q2=q(2);
dq1=dq(1);dq2=dq(2);
c1=cos(q1);s1=sin(q1);
c2=cos(q2);s2=sin(q2);
c12=cos(q1+q2);s12=sin(q1+q2);

% C Matric components
C11=0;
C12=-a1*a2*s2*m2*(dq1+dq2/2);
C21=a1/2*a2*s2*m2*(dq1+dq2/2);
C22=-a1/4*a2*m2*s2*dq1;

% C Matrix
C=[C11 C12 
    C21 C22];
end

