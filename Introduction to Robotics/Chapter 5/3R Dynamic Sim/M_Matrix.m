%% 
% 	An Introduction to Robotics, 
%   By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
%   Copyright ARAS @2023
%
%   This function generates the Mass Matrix of the 3R Robot
%   at configuration q
%
function [M] = M_Matrix(q)

%% Load Physical Parameters of Robot
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
%% Mass Matrix

M11=(m1/3+m2+m3)*a1^2+(m2/3+m3)*a2^2+m3*a3^2+...
    (m2+2*m3)*a1*a2*c2+a3*m3*(a1*c23+a2*c3);
M12=(m2/3+m3)*a2^2 + m3*a3*(a3/3+a2*c23/2)+...
    m2/2*a1*a2*c2+m3*a2*(a1*c2+a3*c23);
M13=m3*a3*(a3/3+a2*c23/2+a2*c3/2);
M22=(m2/3+m3)*a2^2+m3*a3^2/3+m3*a2*a3*c3;
M23=m3*a3*(a3/3+a2*c3/2);
M33=m3*a3^2/3;

M=[M11  M12  M13;
    M12 M22 M23
    M13 M23 M33];  

end

