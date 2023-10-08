%% 
%   Parallel Robots: Mechanics and Control
%   Copyright Hamid D. Taghirad 2013
%
%   This program generates the feedback forces exerted on the      
%   moving Platform by the PD control law in joint space
%
function [M_Dynamic_Mats] = Control(t,X_s,SP,KC,M_Dynamic_Mats,P_Dynamic_Mats)

q  = KC.l'; dq = KC.ldot';

%  add noise to measurements 

q  = q  + SP.noise_q*rand ;
dq = dq + SP.noise_dq*rand;

% use inverse kinematics to find the joint variables

[xd,dxd]=TP_cubic_s(t, SP);
KCD  = Kinematic_Configuration(xd,dxd,SP);
qd = KCD.l'; dqd = KCD.ldot';

M_Dynamic_Mats.F = zeros(6,1) ;         % disturbance force
%
% interpolates disturbance inputs from its given the time history 
%
M_Dynamic_Mats.F(1,1)=interp1(SP.time,SP.Fd(:,1),t); 
M_Dynamic_Mats.F(2,1)=interp1(SP.time,SP.Fd(:,2),t);
M_Dynamic_Mats.F(3,1)=interp1(SP.time,SP.Fd(:,3),t);
M_Dynamic_Mats.F(4,1)=interp1(SP.time,SP.Fd(:,4),t);
M_Dynamic_Mats.F(5,1)=interp1(SP.time,SP.Fd(:,5),t);
M_Dynamic_Mats.F(6,1)=interp1(SP.time,SP.Fd(:,6),t);

M_Dynamic_Mats.CF = zeros(6,1) ;        % caresian force
M_Dynamic_Mats.JF = zeros(SP.n,1) ;     % actuator force

M_Dynamic_Mats.PD=SP.KP*(qd-q)+SP.KV*(dqd-dq);
M_Dynamic_Mats.JF=M_Dynamic_Mats.PD;
M_Dynamic_Mats.CF=(P_Dynamic_Mats.JT*M_Dynamic_Mats.JF);
M_Dynamic_Mats.F = M_Dynamic_Mats.F + M_Dynamic_Mats.CF;

end