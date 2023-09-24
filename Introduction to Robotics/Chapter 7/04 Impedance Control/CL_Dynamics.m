%% 
% 	An Introduction to Robotics, 
%   By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
%   Copyright ARAS @2023
%
%   This function is used in Impedance control of the 3R planar robot
%   while interacting with environment in task space
%
function [cost] = CL_Dynamics(t,qs,SP)

q   = qs(1:3) ;   qd  = qs(4:6) ;
[yd, dyd, d2yd]=TP_quintic(t, SP);          % project to joint space
ys=[yd;dyd;d2yd];       % Augmented State
Qs=IK(ys, SP); Qd = Qs(1:3); 

[Dynamic_Mats] = Dynamic_Matrices(qs, SP) ;
[Dynamic_Tau] = Impedance_Control(t,qs,SP) ;

cost = [qd ;    % \dot{q}
        pinv(Dynamic_Mats.M)*(Dynamic_Tau.total ...
                        -Dynamic_Mats.C*qd-Dynamic_Mats.G); %\ddot{q}
        q;       % \int{q}         
        Qd       % \int{xd}
        ] ;
end
