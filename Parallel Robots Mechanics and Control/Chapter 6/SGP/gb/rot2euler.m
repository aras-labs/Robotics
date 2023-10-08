%% 
%   Parallel Robots: Mechanics and Control
%   Copyright Hamid D. Taghirad 2013
%
%       This program transforms rotation matrix to Euler angles
%
function [th]=rot2euler(R)
%
%   Generating Euler angles from Rotation matrix
%
beta  = atan2(-R(3,1), sqrt(R(1,1)^2+R(2,1)^2));
cbeta = cos(beta);

if cbeta ~= 0,
    gamma = atan2(R(2,1)/cbeta, R(1,1)/cbeta);
    alpha = atan2(R(3,2)/cbeta, R(3,3)/cbeta);
elseif beta == pi/2;
    gamma = 0;
    alpha = atan2(R(1,2),R(2,2));
else
    gamma = 0;
    alpha = atan2(-R(1,2),R(2,2));
end,

th=[alpha;beta;gamma];