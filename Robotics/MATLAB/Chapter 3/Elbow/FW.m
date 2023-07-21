%% 
% 	An Introduction to Robotics, 
%   By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
%   Copyright ARAS @20230
%
%   This program generates Forward Kinematics 
%   the Elbow Mainpulator
%%
function [P,R] = FW(theta1,theta2,theta3,Structual_Parameters)

L2=Structual_Parameters(1);
L3=Structual_Parameters(2);

P=[1.0*cos(theta1)*(L3*cos(theta2 + theta3) + L2*cos(theta2));1.0*sin(theta1)*(L3*cos(theta2 + theta3) + L2*cos(theta2));1.0*L3*sin(theta2 + theta3) + 1.0*L2*sin(theta2)];           
R=[1.0*cos(theta2 + theta3)*cos(theta1),-1.0*sin(theta2 + theta3)*cos(theta1),1.0*sin(theta1);1.0*cos(theta2 + theta3)*sin(theta1), -1.0*sin(theta2 + theta3)*sin(theta1),-1.0*cos(theta1);1.0*sin(theta2 + theta3), 1.0*cos(theta2 + theta3),0];
end

