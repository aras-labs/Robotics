%% 
% 	An Introduction to Robotics, 
%   By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
%   Copyright ARAS @20230
%
%   This program Defines Screw Parameters
%%
function [S] = SR(s__x,s__y,s__z,s__ox,s__oy,s__oz,theta,t)
S=[(s__x ^ 2 - 1) * (0.1e1 - cos(theta)) + 0.1e1 s__x * s__y * (0.1e1 - cos(theta)) - s__z * sin(theta) s__x * s__z * (0.1e1 - cos(theta)) + s__y * sin(theta) (t * s__x) - s__ox * (s__x ^ 2 - 1) * (0.1e1 - cos(theta)) - s__oy * (s__x * s__y * (0.1e1 - cos(theta)) - s__z * sin(theta)) - s__oz * (s__x * s__z * (0.1e1 - cos(theta)) + s__y * sin(theta)); s__x * s__y * (0.1e1 - cos(theta)) + s__z * sin(theta) (s__y ^ 2 - 0.1e1) * (0.1e1 - cos(theta)) + 0.1e1 s__y * s__z * (0.1e1 - cos(theta)) - s__x * sin(theta) t * s__y - s__ox * (s__x * s__y * (0.1e1 - cos(theta)) + s__z * sin(theta)) - s__oy * (s__y ^ 2 - 0.1e1) * (0.1e1 - cos(theta)) - s__oz * (s__y * s__z * (0.1e1 - cos(theta)) - s__x * sin(theta)); s__x * s__z * (0.1e1 - cos(theta)) - s__y * sin(theta) s__y * s__z * (0.1e1 - cos(theta)) + s__x * sin(theta) (s__z ^ 2 - 0.1e1) * (0.1e1 - cos(theta)) + 0.1e1 t * s__z - s__ox * (s__x * s__z * (0.1e1 - cos(theta)) - s__y * sin(theta)) - s__oy * (s__y * s__z * (0.1e1 - cos(theta)) + s__x * sin(theta)) - s__oz * (s__z ^ 2 - 0.1e1) * (0.1e1 - cos(theta)); 0 0 0 1;];
end

