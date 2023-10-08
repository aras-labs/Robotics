%% 
%   Parallel Robots: Mechanics and Control
%   Copyright Hamid D. Taghirad 2013
%
%   This program generates the closed-loop dynamic formulation
%   of the Stewart-Gough platform
%
function [cost] = CL_Dynamic_Equation(t,X_s,Struct_Param)

%disp(t)             % Too keep track of integration time
X = X_s(1:6) ;
Xdot = X_s(7:12) ;

[Kinematic_Conf] = Kinematic_Configuration(X,Xdot,Struct_Param) ;
[P_Dynamic_Mats] = Parts_Dynamic_Matrixes(Struct_Param,Kinematic_Conf) ;
[M_Dynamic_Mats] = Manipulator_Dynamic_Matrixes(Struct_Param,P_Dynamic_Mats) ;
[M_Dynamic_Mats] = Control(t,X_s,Struct_Param,Kinematic_Conf,M_Dynamic_Mats,P_Dynamic_Mats) ;

cost = [Xdot 
    inv(M_Dynamic_Mats.M)*(M_Dynamic_Mats.F-M_Dynamic_Mats.C*Xdot-M_Dynamic_Mats.G)
    M_Dynamic_Mats.Fm
    M_Dynamic_Mats.Fd] ;
end
