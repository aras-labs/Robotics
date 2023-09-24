%% 
% 	An Introduction to Robotics, 
%   By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
%   Copyright ARAS @2023
%
%   This function generates the feedback torques based on     
%   PD control law 
%
function [Dynamic_Tau] = PD_Control(t,qs,SP)

q  = qs(1:3) ;
dq = qs(4:6) ;

%  add noise to measurements 

q  = q  + SP.noise_q*randn;
dq = dq + SP.noise_dq*randn;

[xd,dxd]=TP_quintic(t, SP);
Dynamic_Tau.taud = zeros(3,1) ;         % disturbance force
%
% interpolates disturbance inputs from its given time history 
%
Dynamic_Tau.taud(1,1)=interp1(SP.time,SP.taud(:,1),t); % disturbance
Dynamic_Tau.taud(2,1)=interp1(SP.time,SP.taud(:,2),t);
Dynamic_Tau.taud(3,1)=interp1(SP.time,SP.taud(:,3),t);

Dynamic_Tau.PD=SP.Kp*(xd-q)+SP.Kd*(dxd-dq);           % PD control law
Dynamic_Tau.tau = Dynamic_Tau.PD;
Dynamic_Tau.total = Dynamic_Tau.PD + Dynamic_Tau.taud;

end         % end of main function