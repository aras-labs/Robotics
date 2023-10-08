%% 
%   Parallel Robots: Mechanics and Control
%   Copyright Hamid D. Taghirad 2013
%
%   This program generates the feedback forces exerted on the      
%   moving Platform by the PD control law + Feedforward term
%
function [M_Dynamic_Mats] = FF_Control(t,X_s,SP,KC,M_Dynamic_Mats,P_Dynamic_Mats)

x  = X_s(1:3) ;
dx = X_s(4:6) ;

%  add noise to measurements 

x  = x  + SP.noise_x*rand ;
dx = dx + SP.noise_dx*rand;

[xd,dxd,d2xd]=TP_cubic_s(t, SP);
% Xd=[xd;dxd];
% Xd_dot=[dxd;d2xd];

M_Dynamic_Mats.F = zeros(3,1) ;         % disturbance force
%
% interpolates disturbance inputs from its given the time history 
%
M_Dynamic_Mats.F(1,1)=interp1(SP.time,SP.Fd(:,1),t); % Experimental wind disturbance
M_Dynamic_Mats.F(2,1)=interp1(SP.time,SP.Fd(:,2),t);
M_Dynamic_Mats.F(3,1)=interp1(SP.time,SP.Fd(:,3),t);

M_Dynamic_Mats.FF = zeros(3,1) ;        % Feedforward force
M_Dynamic_Mats.CF = zeros(3,1) ;        % caresian force
M_Dynamic_Mats.JF = zeros(SP.n,1) ;     % actuator force

SP   = SP_Perturbed (SP) ;                  % Use perturbed model
KCD  = Kinematic_Configuration(xd,dxd,SP); % Kinematics configuration parameters at desired trajectory
PDMD = Parts_Dynamic_Matrixes(SP,KCD);
MDMD = Manipulator_Dynamic_Matrixes(SP,PDMD);

M_Dynamic_Mats.FF=MDMD.M*d2xd+MDMD.C*dxd+MDMD.G;         % FF control law
M_Dynamic_Mats.PD=SP.KP*(xd-x)+SP.KV*(dxd-dx);           % PD control law
M_Dynamic_Mats.CF=M_Dynamic_Mats.FF+M_Dynamic_Mats.PD;   % final control law
M_Dynamic_Mats.F = M_Dynamic_Mats.F + M_Dynamic_Mats.CF;

end         % end of main function