%% 
% 	An Introduction to Robotics, 
%   By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
%   Copyright ARAS @2023
%
%   This function generates the feedback torques based on
%   PID control law.    

%
function [Dynamic_Tau] = PID_Control(t,qs,SP)

q  = qs(1:3) ;
dq = qs(4:6) ;
qi = qs(7:9) ;
xdi= qs(10:12);

%
% interpolates noise inputs from its given time history 
%
qnoise=interp1(SP.noise(:,1),SP.noise(:,2),t)*SP.Isn;  % noise on q
dqnoise=interp1(SP.noise(:,1),SP.noise(:,3),t)*SP.Isn; % noise on dq

%  add noise to measurements 
qn  = q  + [1; 1; 1]*0.1*qnoise;
dqn = dq + [1; 1; 1]*0.1*dqnoise;

[xd,dxd]=TP_quintic(t, SP);
Dynamic_Tau.taud = zeros(3,1) ;         % disturbance force
%
% interpolates disturbance inputs from its given time history 
%
Dynamic_Tau.taud(1,1)=interp1(SP.time,SP.taud(:,1),t)*SP.Isd;
Dynamic_Tau.taud(2,1)=interp1(SP.time,SP.taud(:,2),t)*SP.Isd;
Dynamic_Tau.taud(3,1)=interp1(SP.time,SP.taud(:,3),t)*SP.Isd;
%
% PID control law
%
Dynamic_Tau.PID=SP.Ki*(xdi-qi)+SP.Kp*(xd-qn)+SP.Kd*(dxd-dqn);    
if SP.Istau == 1   % Actuator Saturation Applies
  if    Dynamic_Tau.PID(1) >= SP.tau_limit(1)
    Dynamic_Tau.PID(1) = SP.tau_limit(1);
  elseif Dynamic_Tau.PID(1) <= -SP.tau_limit(1)
    Dynamic_Tau.PID(1) = -SP.tau_limit(1);
  end
  if    Dynamic_Tau.PID(2) >= SP.tau_limit(2)
    Dynamic_Tau.PID(2) = SP.tau_limit(2);
  elseif Dynamic_Tau.PID(2) <= -SP.tau_limit(2)
    Dynamic_Tau.PID(2) = -SP.tau_limit(2);
  end
  if    Dynamic_Tau.PID(3) >= SP.tau_limit(3)
    Dynamic_Tau.PID(3) = SP.tau_limit(3);
  elseif Dynamic_Tau.PID(3) <= -SP.tau_limit(3)
    Dynamic_Tau.PID(3) = -SP.tau_limit(3);
  end
end
%
%   Total torque applied to the robot
%
Dynamic_Tau.tau = Dynamic_Tau.PID;                  % Actuator Torques
Dynamic_Tau.total = Dynamic_Tau.PID + Dynamic_Tau.taud; % Add Disturbance

end         % end of main function