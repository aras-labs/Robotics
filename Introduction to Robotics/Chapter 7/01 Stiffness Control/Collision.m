%% 
% 	An Introduction to Robotics, 
%   By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
%   Copyright ARAS @2023
%
%   This program checks if the 3R robot end effector 
%   will collide with a flat obect and finds the 
%   collision force Fe
%
function [Fe,xc,yc] = Collision(Kin, SP)

%  A Flat surface with (y-yl)-sl(x-xl)=0
%  is considered within the workspace of robot. 

x=Kin.x3;    y=Kin.y3;  phi=Kin.p3; 
xw=Kin.x2;   yw=Kin.y2; sw=tan(phi);
dx=Kin.dx;  dy=Kin.dy; 
xl= SP.xl; yl=SP.yl; sl=SP.sl;
Fe = zeros(3,1);
%if sw ~= sl;    % find the collision point if any
    xc=(yl-yw +sw*xw-sl*xl)/(sw-sl);
    yc=yl+sl*(xc-xl);
    if xc < x  && yc < y    % collision will occur 
    delX=   [x-xc; y-yc; 0];
    deldX=  [dx;dy;0];
    Fe = -(SP.Ke*delX + SP.Ce*deldX);
  end 
%end
