%% 
%   Parallel Robots: Mechanics and Control
%   Copyright Hamid D. Taghirad 2013
%
%   This program generates the feedback forces exerted on the      
%   moving Platform by  Inverse Dynamics control in an adaptive structure
%
function [M_Dynamic_Mats] = AIDC_Control(t,X_s,SP,KC,M_Dynamic_Mats,P_Dynamic_Mats)

x  = X_s(1:3) ;
dx = X_s(4:6) ;
th = X_s(7:9) ;

%  add noise to measurements 

x  = x  + SP.noise_x*rand ;
dx = dx + SP.noise_dx*rand;

[xd,dxd,d2xd]=TP_cubic_s(t, SP);

%        
% Need accelerations for adaptive law
%
%X2d = Acceleration(t,X_s,SP,KC,M_Dynamic_Mats); %Use Dynamics formulation
X2d = d2xd;             % use desired acceleration as true acceleration
d2x = X2d ;             % position and orientation acceleration
d2p = [X2d(1:2);0];     % position acceleration
d2o = [0;0;X2d(3)];     % orientation acceleration

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
M_Dynamic_Mats.Phi = eye(3,3) ;

% Adaptive components

C1 = d2p - [SP.g(1);SP.g(2);0];  
C2 = d2o ;
C3 =  M_Dynamic_Mats.Mr * d2x + M_Dynamic_Mats.Cr * dx + M_Dynamic_Mats.Gr;
M_Dynamic_Mats.Y = [C1 C2 C3];


SP   = SP_update (SP, th) ;                 % Use updated model
KCD  = Kinematic_Configuration(x,dx,SP);    % Kinematics configuration 
PDMD = Parts_Dynamic_Matrixes(SP,KCD);
MDMD = Manipulator_Dynamic_Matrixes(SP,PDMD);

M_Dynamic_Mats.Phi = inv(MDMD.M)* M_Dynamic_Mats.Y;
M_Dynamic_Mats.PD  =  MDMD.M*(SP.KP*(xd-x)+SP.KV*(dxd-dx));   % PD part
M_Dynamic_Mats.CF  =  MDMD.M*(d2xd) + M_Dynamic_Mats.PD + ...
    MDMD.C*dx + MDMD.G;                                  % IDC control law
M_Dynamic_Mats.F = M_Dynamic_Mats.F + M_Dynamic_Mats.CF;

end         % end of main function