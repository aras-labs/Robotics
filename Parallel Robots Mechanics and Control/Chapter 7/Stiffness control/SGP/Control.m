%% 
%   Parallel Robots: Mechanics and Control
%   Copyright Hamid D. Taghirad 2013
%
%   This program generates the feedback forces exerted on the
%   moving Platform by the IDC stiffness control 
%
function [M_Dynamic_Mats] = Control(t,X_s,SP,KC,M_Dynamic_Mats,P_Dynamic_Mats)

x  = X_s(1:6) ;
dx = X_s(7:12) ;

%  add noise to measurements 

x  = x  + SP.noise_x*rand ;
dx = dx + SP.noise_dx*rand;

[xd,dxd,d2xd]=TP_cubic_s(t, SP);
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

%
%  A lnear plane with nx(x_e-xp)+ ny(y_e-yp)+ nz(z_e-zp)=0
%  is considered within the workspace of robot. 
%
xx=x(1); yy=x(2); zz=x(3);              % dummy variables
xp=SP.xp(1); yp=SP.xp(2); zp=SP.xp(3);  % dummy variables
nx=SP.nl(1); ny=SP.nl(2); nz=SP.nl(3);  % dummy variables

%  First check if any collision occurs

if nx*(xx-xp)+ ny*(yy-yp)+nz*(zz-zp)> 0,
    
    % Find the elastic deformation
    
    xe=[xp-(ny*(yy-yp)+nz*(zz-zp))/nx; ...
        yp-(nx*(xx-xp)+nz*(zz-zp))/ny; ...
        zp-(ny*(yy-yp)+nx*(xx-xp))/nz; ...
        0;0;0];
    M_Dynamic_Mats.Fe = SP.Ke * (x-xe) + SP.Ce * dx;
else
    M_Dynamic_Mats.Fe = zeros(6,1);
end

SP   = SP_Perturbed (SP) ;                 % Use perturbed model
KCD  = Kinematic_Configuration(xd,dxd,SP); % Kinematics configuration parameters at desired trajectory
PDMD = Parts_Dynamic_Matrixes(SP,KCD);
MDMD = Manipulator_Dynamic_Matrixes(SP,PDMD);

M_Dynamic_Mats.PD=MDMD.M*(SP.KP*(xd-x)+SP.KV*(dxd-dx));           % PD control law
M_Dynamic_Mats.CF=MDMD.M*(d2xd) + M_Dynamic_Mats.PD + MDMD.C*dx + MDMD.G;
                                                                  % IDC control law
M_Dynamic_Mats.F  = M_Dynamic_Mats.F -  M_Dynamic_Mats.Fe + M_Dynamic_Mats.CF;
M_Dynamic_Mats.JF=(pinv(P_Dynamic_Mats.JT)*M_Dynamic_Mats.CF)';

end