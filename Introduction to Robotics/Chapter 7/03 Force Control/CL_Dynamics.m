%% 
% 	An Introduction to Robotics, 
%   By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
%   Copyright ARAS @2023
%
%   This function is used in Force control of the 3R planar robot
%   while interacting with environment
%
function [cost] = CL_Dynamics(t,qs,SP)

q   = qs(1:3) ;   qd  = qs(4:6) ;
%tauei = qs(7:9) ; taudi= qs(10:12);

[Dynamic_Mats] = Dynamic_Matrices(qs, SP) ;
[Tau] = Force_Control(t,qs,SP) ;

cost = [qd ;    % \dot{q}
        pinv(Dynamic_Mats.M)*(Tau.total ...
                 -Dynamic_Mats.C*qd - Dynamic_Mats.G ); %\ddot{q}
        Tau.taum ;        % integal of measured torque
        Tau.taud ;        % integal of desired torque
        ] ;
end
