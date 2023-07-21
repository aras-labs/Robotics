%% 
%   Robotics Course, Professor: Prof. Hamid D. Taghirad
%   Aras.kntu.ac.ir/education/robotics
%   Copyright Ali.Hassani 2020
%
%   This program Defines DH Parameters
%%
function [T] = DH(a,alpha,d,theta)
T=[cos(theta) -sin(theta) * cos(alpha) sin(theta) * sin(alpha) a * cos(theta); sin(theta) cos(theta) * cos(alpha) -cos(theta) * sin(alpha) a * sin(theta); 0 sin(alpha) cos(alpha) d; 0 0 0 1;];
end

