%% 
% 	An Introduction to Robotics, 
%   By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
%   Copyright ARAS @2023
%
%   This program generates the feedback torques based on     
%   Direct Force Control  
%
function [Tau] = Force_Control(t,qs,SP)

q  = qs(1:3) ; dq = qs(4:6) ; Q  = qs(1:6);  
taumi = qs(7:9) ; taudi= qs(10:12);

%
% interpolates noise inputs from its given time history 
%
noise=interp1(SP.noise(:,1),SP.noise(:,2),t)*SP.Isn;  % noise on q
dnoise=interp1(SP.noise(:,1),SP.noise(:,3),t)*SP.Isn; % noise on dq

%  add noise to measurements 
qn  = q  + [1; 1; 1]*0.1*noise; 
dqn = dq + [1; 1; 1]*0.1*dnoise;

%   Force Trajectory planning in task space

[Fd]=TP_quintic(t, SP);          
[Kin]=Kinematics(qs, SP);        % FK and Jacobian
Jac=Kin.J; Tau.taud= Jac'*Fd;
[Dynamic_Mats] = Dynamic_Matrices(qs, SP) ;
[Tau.Fe, Tau.xc, Tau.yc] = Collision(Kin, SP);    % Collision Force

%
% interpolates disturbance inputs from its given time history 
%
Tau.dist = zeros(3,1) ;         % disturbance torque
Tau.dist(1,1)=interp1(SP.time,SP.taud(:,1),t)*SP.Isd;
Tau.dist(2,1)=interp1(SP.time,SP.taud(:,2),t)*SP.Isd;
Tau.dist(3,1)=interp1(SP.time,SP.taud(:,3),t)*SP.Isd;
%
% Force control law
%
Tau.taue = Jac'* Tau.Fe;
Tau.taum = Tau.taue + [1; 1; 1]*dnoise;

Tau.qa = - pinv(SP.Kp) *(SP.Ktaup*(Tau.taud-Tau.taum) + SP.Ktaui*(taudi-taumi));
Tau.a  = SP.Kp*(Tau.qa - qn) - SP.Kd*dqn;

Tau.FL = Dynamic_Mats.Mh*Tau.a + Dynamic_Mats.Ch*dq + Dynamic_Mats.Gh; 
Tau.tau = Tau.FL + Tau.taum;

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
Tau.total = Tau.tau + Tau.dist - Tau.taue; % Add dist and env forces

end         % end of main function