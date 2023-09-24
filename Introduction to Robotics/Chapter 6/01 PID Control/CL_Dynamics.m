%% 
% 	An Introduction to Robotics, 
%   By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
%   Copyright ARAS @2023
%
%   This function generates the closed-loop dynamic formulation
%   of the 3R planar robot with PID controller
%
function [cost] = CL_Dynamics(t,qs,SP)

%disp(t)             % Too keep track of integration time
q   = qs(1:3) ;
qd  = qs(4:6) ;
xd  = TP_quintic(t, SP);

[Dynamic_Mats] = Dynamic_Matrices(qs, SP) ;
[Dynamic_Tau] = PID_Control(t,qs,SP) ;

cost = [qd ;    % \dot{q}
        pinv(Dynamic_Mats.M)*(Dynamic_Tau.total ...
                        -Dynamic_Mats.C*qd-Dynamic_Mats.G); %\ddot{q}
        q;       % \int{q}         
        xd       % \int{xd}
        ] ;
end
