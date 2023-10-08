%% 
%   Parallel Robots: Mechanics and Control
%   Copyright Hamid D. Taghirad 2013
%
%   This program generates the feedback forces exerted on the
%   Stewart-Gough Platform by the PD control law 
%
function [M_Dynamic_Mats] = PD_Control(t,X_s,Struct_Param,Kinematic_Conf,M_Dynamic_Mats,P_Dynamic_Mats)

x  = X_s(1:6) ;
dx = X_s(7:12) ;

[xd,dxd]=TP_cubic_s(t, Struct_Param);

CF=Struct_Param.KP*(xd-x)+Struct_Param.KV*(dxd-dx);
tau=(inv(P_Dynamic_Mats.JT)*CF)';

M_Dynamic_Mats.F = zeros(6,1) ;

for i = 1:Struct_Param.n
    F_temp = [tau(:,i)*Kinematic_Conf.s(:,i)
              Kinematic_Conf.E'*Kinematic_Conf.B_x(:,:,i)*tau(:,i)*Kinematic_Conf.s(:,i)] ;
    M_Dynamic_Mats.F = M_Dynamic_Mats.F + F_temp ;
end
end