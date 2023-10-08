%% 
%   Parallel Robots: Mechanics and Control
%   Copyright Hamid D. Taghirad 2013
%
%       This program transforms rotation matrix to screw coordinates
%
function [theta]=rot2sc(R)
%
%   Generating Rotation matrix from screw axis
%
th=acos((trace(R)-1)/2);
sth=sin(th);

s_x=(R(3,2)-R(2,3))/(2*sth);
s_y=(R(1,3)-R(3,1))/(2*sth);
s_z=(R(2,1)-R(1,2))/(2*sth);
theta=[s_x;s_y;s_z]*th;