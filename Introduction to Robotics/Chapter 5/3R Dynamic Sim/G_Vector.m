%% 
% 	An Introduction to Robotics, 
%   By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
%   Copyright ARAS @2023
%
%   This function generates the gravity vector of the 3R Robot
%   at configuration q
%
function [G] = G_Vector(q)

% Define Numeric Variables
run("Parameters.m");
a1=a(1);a2=a(2);a3=a(3);
m1=m(1);m2=m(2);m3=m(3);
q1=q(1);q2=q(2);q3=q(3);
c1=cos(q1);s1=sin(q1);
c2=cos(q2);s2=sin(q2);
c3=cos(q3);s3=sin(q3);
c12=cos(q1+q2);s12=sin(q1+q2);
c23=cos(q2+q3);s23=sin(q2+q3);
c123=cos(q1+q2+q3);s123=sin(q1+q2+q3);

%% G Vector
G=[m1/2*a1*c1+m2/2*(a1*c1+a2*c12)+m3*(a2*c12+a3/2*c123);
    m2/2*a2*c12+m3*(a2*c12+a3/2*c123);
    m3/2*a3*c123]*g;
end

