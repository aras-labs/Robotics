%% 
%   Parallel Robots: Mechanics and Control
%   Copyright Hamid D. Taghirad 2013
%
%       This program transforms screw coordinates to rotation matrix
%
function R=sc2rot(s,th)
%
%   Generating Rotation matrix from screw axis
%
s=s./norm(s);
s_x=s(1);
s_y=s(2);
s_z=s(3);
sth=sin(th);
cth=cos(th); ;
vth=1-cth;
R=[
s_x^2*vth + cth, s_x*s_y*vth-s_z*sth, s_x*s_z*vth+s_y*sth 
s_y*s_x*vth+s_z*sth, s_y^2*vth+cth, s_y*s_z*vth-s_x*sth 
s_z*s_x*vth-s_y*sth, s_z*s_y*vth+s_x*sth, s_z^2*vth+ cth
];