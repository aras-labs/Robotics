%% 
% 	An Introduction to Robotics, 
%   By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
%   Copyright ARAS @2023
%
%   This program generates the feedback torques based on     
%   Stiffness control law 
%
function [Tau] = Stiffness_Control(t,qs,SP)

q  = qs(1:3) ; dq = qs(4:6) ; Q  = qs(1:6);  
qi = qs(7:9) ; qdi= qs(10:12);

%
% interpolates noise inputs from its given time history 
%
qnoise=interp1(SP.noise(:,1),SP.noise(:,2),t)*SP.Isn;  % noise on q
dqnoise=interp1(SP.noise(:,1),SP.noise(:,3),t)*SP.Isn; % noise on dq

%  add noise to measurements 
qn  = q  + [1; 1; 1]*0.1*qnoise;
dqn = dq + [1; 1; 1]*0.1*dqnoise;

%   Trajectory planning in task space

[yd, dyd, d2yd]=TP_quintic(t, SP);          % project to joint space
ys=[yd;dyd;d2yd];       % Augmented State
Qs=IK(ys, SP);
qd = Qs(1:3);  dqd= Qs(4:6);  d2qd=Qs(7:9);

[Kin]=Kinematics(qs, SP);        % FK and Jacobian
[Dynamic_Mats] = Dynamic_Matrices(qs, SP) ;
Tau.taud = zeros(3,1) ;         % disturbance force
[Tau.Fe, Tau.xc, Tau.yc] = Collision(Kin, SP);    % Collision Force
Jac=Kin.J;
%
% interpolates disturbance inputs from its given time history 
%
Tau.taud(1,1)=interp1(SP.time,SP.taud(:,1),t)*SP.Isd;
Tau.taud(2,1)=interp1(SP.time,SP.taud(:,2),t)*SP.Isd;
Tau.taud(3,1)=interp1(SP.time,SP.taud(:,3),t)*SP.Isd;
%
% PID control law
%
Tau.PID=(SP.Ki*(qdi-qi)+SP.Kp*(qd-qn)+SP.Kd*(dqd-dqn))*1; 
%   Normalized K's
% Ki=SP.Ki*Dynamic_Mats.Mh;
% Kd=SP.Kd*Dynamic_Mats.Mh;
% Kp=SP.Kp*Dynamic_Mats.Mh;
% Tau.PID=Ki*(qdi-qi)+Kp*(qd-qn)+Kd*(dqd-dqn);
aq= d2qd + Tau.PID;
Tau.FL=Dynamic_Mats.Mh*aq+Dynamic_Mats.Ch*dqd+Dynamic_Mats.Gh;
Tau.tau=Tau.PID + Tau.FL;
Tau.taue=-Jac'*Tau.Fe;
if SP.Istau == 1   % Actuator Saturation Applies
  if    Tau.tau(1) >= SP.tau_limit(1)
    Tau.tau(1) = SP.tau_limit(1);
  elseif Tau.tau(1) <= -SP.tau_limit(1)
    Tau.tau(1) = -SP.tau_limit(1);
  end
  if    Tau.tau(2) >= SP.tau_limit(2)
    Tau.tau(2) = SP.tau_limit(2);
  elseif Tau.tau(2) <= -SP.tau_limit(2)
    Tau.tau(2) = -SP.tau_limit(2);
  end
  if    Tau.tau(3) >= SP.tau_limit(3)
    Tau.tau(3) = SP.tau_limit(3);
  elseif Tau.tau(3) <= -SP.tau_limit(3)
    Tau.tau(3) = -SP.tau_limit(3);
  end
end
%
%   Total torque applied to the robot
%
Tau.total = Tau.tau + Tau.taud - Tau.taue; % Add dist and elastic force

end         % end of main function