%% 
%   Parallel Robots: Mechanics and Control
%   Copyright Hamid D. Taghirad 2013
%
%   This program generates the feedback forces exerted on the      
%   moving Platform by the PD control law 
%
function [M_Dynamic_Mats] = PD_Control(t,X_s,Struct_Param,Kinematic_Conf,M_Dynamic_Mats,P_Dynamic_Mats)


% Note my x(4:6) is screw-based, while Reza's are Euler angles
x  = X_s(1:6) ;
dx = X_s(7:12) ;
%  add noise to measurements 

x  = x  + Struct_Param.noise_x*rand ;
dx = dx + Struct_Param.noise_dx*rand;

[xd,dxd]=TP_cubic_euler(t, Struct_Param);

M_Dynamic_Mats.F = zeros(6,1) ;
M_Dynamic_Mats.JF = zeros(6,1) ;

% interpolates disturbance inputs from its given the time history 
%
M_Dynamic_Mats.F(1,1)=interp1(Struct_Param.time,Struct_Param.Fd(:,1),t); 
M_Dynamic_Mats.F(2,1)=interp1(Struct_Param.time,Struct_Param.Fd(:,2),t);
M_Dynamic_Mats.F(3,1)=interp1(Struct_Param.time,Struct_Param.Fd(:,3),t);
M_Dynamic_Mats.F(4,1)=interp1(Struct_Param.time,Struct_Param.Fd(:,4),t);
M_Dynamic_Mats.F(5,1)=interp1(Struct_Param.time,Struct_Param.Fd(:,5),t);
M_Dynamic_Mats.F(6,1)=interp1(Struct_Param.time,Struct_Param.Fd(:,6),t);

M_Dynamic_Mats.PD=Struct_Param.KP*(xd-x)+Struct_Param.KV*(dxd-dx);
M_Dynamic_Mats.CF=M_Dynamic_Mats.PD;
M_Dynamic_Mats.JF=(pinv(P_Dynamic_Mats.JT)*M_Dynamic_Mats.CF)';
M_Dynamic_Mats.F = M_Dynamic_Mats.F + M_Dynamic_Mats.CF;

end