%% 
%   Parallel Robots: Mechanics and Control
%   Copyright Hamid D. Taghirad 2013
%
%   This program generates the feedback forces exerted on the      
%   moving Platform by the PD control law + Feedforward term in joint space
%
function [M_Dynamic_Mats] = Control(t,X_s,SP,KC,M_Dynamic_Mats,P_Dynamic_Mats)

q  = KC.l'; dq = KC.ldot';

%  add noise to measurements 

q  = q  + SP.noise_q*rand ;
dq = dq + SP.noise_dq*rand;

[xd,dxd,d2xd]=TP_cubic_s(t, SP);
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

M_Dynamic_Mats.CF = zeros(3,1) ;        % caresian force
M_Dynamic_Mats.JF = zeros(SP.n,1) ;     % actuator force

SP   = SP_Perturbed (SP) ;                  % Use perturbed model
KCP  = Kinematic_Configuration(xd,dxd,SP); % Kinematics configuration parameters at desired trajectory
PDMD = Parts_Dynamic_Matrixes(SP,KCP);
MDMD = Manipulator_Dynamic_Matrixes(SP,PDMD);

M_Dynamic_Mats.FF =  pinv(P_Dynamic_Mats.JT)*(MDMD.M*d2xd+MDMD.C*dxd+MDMD.G);         % FF control law
M_Dynamic_Mats.PD = SP.KP*(qd-q)+SP.KV*(dqd-dq);             % PD control law
M_Dynamic_Mats.JF= M_Dynamic_Mats.PD + M_Dynamic_Mats.FF;
M_Dynamic_Mats.CF= P_Dynamic_Mats.JT * M_Dynamic_Mats.JF ;
M_Dynamic_Mats.F = M_Dynamic_Mats.F + M_Dynamic_Mats.CF;

end