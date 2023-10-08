%% 
%   Parallel Robots: Mechanics and Control
%   Copyright Hamid D. Taghirad 2013
%
%   This program is used for forward dynamics simulation and generates
%   zero feedback force exerted on the Stewart-Gough Platform 
%
function [M_Dynamic_Mats] = Control(Struct_Param,Kinematic_Conf,M_Dynamic_Mats)

tau = 0*ones(1,Struct_Param.n) ;
M_Dynamic_Mats.F = zeros(6,1) ;

for i = 1:Struct_Param.n
    F_temp = [tau(:,i)*Kinematic_Conf.s(:,i)
              Kinematic_Conf.E'*Kinematic_Conf.B_x(:,:,i)*tau(:,i)*Kinematic_Conf.s(:,i)] ;
    M_Dynamic_Mats.F = M_Dynamic_Mats.F + F_temp ;
end
end