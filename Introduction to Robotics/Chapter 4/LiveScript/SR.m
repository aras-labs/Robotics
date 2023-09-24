%% 
% 	An Introduction to Robotics, 
%   By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
%   Copyright ARAS @2023
%
%   This program defines the screw matrix based on the given screw parameters.
%
%   Inputs:
%   - s_x, s_y, s_z: Components of the screw axis
%   - s_ox, s_oy, s_oz: Components of the screw moment
%   - theta: The twist angle
%   - t: The pitch (scalar factor of the screw)
%
%   Output:
%   - S: The screw matrix corresponding to the given parameters.
%
%   Note: The screw matrix is used in robotics to represent the spatial motion 
%   and forces of a rigid body. It combines the components of the screw axis, 
%   screw moment, twist angle, and pitch into a 4x4 matrix.

function [S] = SR(s_x, s_y, s_z, s_ox, s_oy, s_oz, theta, t)
    % Calculate the screw matrix using the provided screw parameters.
    S = [(s_x^2 - 1)*(1 - cos(theta)) + 1, s_x*s_y*(1 - cos(theta)) - s_z*sin(theta), s_x*s_z*(1 - cos(theta)) + s_y*sin(theta), t*s_x - s_ox*(s_x^2 - 1)*(1 - cos(theta)) - s_oy*(s_x*s_y*(1 - cos(theta)) - s_z*sin(theta)) - s_oz*(s_x*s_z*(1 - cos(theta)) + s_y*sin(theta)); ...
         s_x*s_y*(1 - cos(theta)) + s_z*sin(theta), (s_y^2 - 1)*(1 - cos(theta)) + 1, s_y*s_z*(1 - cos(theta)) - s_x*sin(theta), t*s_y - s_ox*(s_x*s_y*(1 - cos(theta)) + s_z*sin(theta)) - s_oy*(s_y^2 - 1)*(1 - cos(theta)) - s_oz*(s_y*s_z*(1 - cos(theta)) - s_x*sin(theta)); ...
         s_x*s_z*(1 - cos(theta)) - s_y*sin(theta), s_y*s_z*(1 - cos(theta)) + s_x*sin(theta), (s_z^2 - 1)*(1 - cos(theta)) + 1, t*s_z - s_ox*(s_x*s_z*(1 - cos(theta)) - s_y*sin(theta)) - s_oy*(s_y*s_z*(1 - cos(theta)) + s_x*sin(theta)) - s_oz*(s_z^2 - 1)*(1 - cos(theta)); ...
         0, 0, 0, 1];
end
