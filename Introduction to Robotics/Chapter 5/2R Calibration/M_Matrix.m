%% 
% 	An Introduction to Robotics, 
%   By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
%   Copyright ARAS @2023
%
%   This function generates the Mass Matrix of the 2R Robot
%   at configuration q
%
function [M] = M_Matrix(q)

%% Load Physical Parameters of Robot
% Define Numeric Variables
run("Parameters.m");
a1=a(1);a2=a(2);
m1=m(1);m2=m(2);
q1=q(1);q2=q(2);
c1=cos(q1);s1=sin(q1);
c2=cos(q2);s2=sin(q2);
c12=cos(q1+q2);s12=sin(q1+q2);

%% Mass Matrix

M11=m1/3*a1^2+m2*(a1^2+a1*a2*c2+1/3*a2^2);
M12=m2*(a1/2*a2*c2+1/3*a2^2);
M22=m2/3*a2^2;
M=[M11  M12;
    M12 M22];
end

