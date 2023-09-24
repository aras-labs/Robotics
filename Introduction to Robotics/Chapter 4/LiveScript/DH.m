%% 
% 	An Introduction to Robotics, 
%   By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
%   Copyright ARAS @2023
%
%   This program defines the Denavit-Hartenberg (DH) transformation matrix.
%
%   Inputs:
%   - a: The link length (length between the Z axes)
%   - alpha: The link twist (rotation about the X axes)
%   - d: The link offset (distance between the X axes)
%   - theta: The joint angle (rotation about the Z axes)
%
%   Output:
%   - T: The DH transformation matrix corresponding to the given parameters.
%
%   Note: The DH convention is commonly used in robotics to model the kinematics 
%   of robot manipulators. This function calculates the 4x4 transformation matrix 
%   based on the DH parameters, which can be used to determine the position and 
%   orientation of a robot's end-effector relative to its base frame.

function [T] = DH(a, alpha, d, theta)
    % Calculate the DH transformation matrix using the provided DH parameters.
    T = [cos(theta) -sin(theta) * cos(alpha) sin(theta) * sin(alpha) a * cos(theta); ...
         sin(theta) cos(theta) * cos(alpha) -cos(theta) * sin(alpha) a * sin(theta); ...
         0 sin(alpha) cos(alpha) d; ...
         0 0 0 1];
end
