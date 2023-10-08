function [M_Dynamic_Mats] = IC_Control(t,X_s,SP,KC,M_Dynamic_Mats,P_Dynamic_Mats)

x  = X_s(1:3) ;
dx = X_s(4:6) ;

%  add noise to measurements 

x  = x  + SP.noise_x*rand ;
dx = dx + SP.noise_dx*rand;

[xd,dxd,d2xd]=TP_cubic_s(t, SP);
% Xd=[xd;dxd];
% Xd_dot=[dxd;d2xd];

M_Dynamic_Mats.Fe = zeros(3,1) ;        % Environment force
M_Dynamic_Mats.F  = zeros(3,1) ;        % disturbance force
M_Dynamic_Mats.CF = zeros(3,1) ;        % caresian force
M_Dynamic_Mats.FL = zeros(3,1) ;        % Feedback linearizing force
M_Dynamic_Mats.Fm = zeros(3,1) ;        % measured force
M_Dynamic_Mats.Fa = zeros(3,1) ;        % auxiliary force
M_Dynamic_Mats.Fimp = zeros(3,1) ;        % auxiliary force
M_Dynamic_Mats.JF = zeros(SP.n,1) ;     % actuator force


%
%  A lnear surface with x_e - xp = sl*(y_e - yp)
%  is considered within the workspace of robot. 
%
%  First check if any collision occurs
xx=x(1);yy=x(2);
if yy-SP.sl*xx > SP.xp(2) - SP.sl * SP.xp(1),
    xe=[SP.xp(1)+(yy-SP.xp(2))/SP.sl;
        SP.xp(2)+(xx-SP.xp(1))*SP.sl;
        0];
    M_Dynamic_Mats.Fe = SP.Ke * (x-xe) + SP.Ce * dx;
    M_Dynamic_Mats.Fm = M_Dynamic_Mats.Fe + SP.noise_F*rand;
else
    M_Dynamic_Mats.Fe = zeros(3,1);
    M_Dynamic_Mats.Fm = M_Dynamic_Mats.Fe;
end

%
% interpolates disturbance inputs from its given the time history 
%
M_Dynamic_Mats.F(1,1)=interp1(SP.time,SP.Fd(:,1),t); % Experimental wind disturbance
M_Dynamic_Mats.F(2,1)=interp1(SP.time,SP.Fd(:,2),t);
M_Dynamic_Mats.F(3,1)=interp1(SP.time,SP.Fd(:,3),t);



SP   = SP_Perturbed (SP) ;               % Use perturbed model
KCD  = Kinematic_Configuration(x,dx,SP); % Kinematics configuration parameters at desired trajectory
PDMD = Parts_Dynamic_Matrixes(SP,KCD);
MDMD = Manipulator_Dynamic_Matrixes(SP,PDMD);

M_Dynamic_Mats.FL = MDMD.C*dx  + MDMD.G + M_Dynamic_Mats.Fm;      % FL force
M_Dynamic_Mats.Fa = SP.Md*d2xd + SP.Cd*(xd-x)+ SP.Kd*(dxd-dx);    % Aux Force
M_Dynamic_Mats.PD = MDMD.M*pinv(SP.Md)*(M_Dynamic_Mats.Fa - M_Dynamic_Mats.Fm);
M_Dynamic_Mats.CF = M_Dynamic_Mats.PD + M_Dynamic_Mats.FL;
M_Dynamic_Mats.F = M_Dynamic_Mats.F - M_Dynamic_Mats.Fe + M_Dynamic_Mats.CF;
M_Dynamic_Mats.Fimp= M_Dynamic_Mats.Fa -SP.Md*pinv(M_Dynamic_Mats.M)* ...
    (M_Dynamic_Mats.CF - M_Dynamic_Mats.C*dx - M_Dynamic_Mats.G - ...
                                        M_Dynamic_Mats.Fe); % For impedance check

end         % end of main function