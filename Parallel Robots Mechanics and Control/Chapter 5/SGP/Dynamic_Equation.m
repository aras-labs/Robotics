%% 
%   Parallel Robots: Mechanics and Control
%   Copyright Hamid D. Taghirad 2013
%
%   This program generates the explicit dynamic formulation
%   of Stewart-Gough Platform
%
function [cost] = Dynamic_Equation(t,X_s,Struct_Param)

X = X_s(1:6) ;
Xdot = X_s(7:12) ;

[Kinematic_Conf] = Kinematic_Configuration(X,Xdot,Struct_Param) ;
[P_Dynamic_Mats] = Parts_Dynamic_Matrixes(Struct_Param,Kinematic_Conf) ;
[M_Dynamic_Mats] = Manipulator_Dynamic_Matrixes(Struct_Param,P_Dynamic_Mats) ;
[M_Dynamic_Mats] = Control(Struct_Param,Kinematic_Conf,M_Dynamic_Mats) ;

cost = [Xdot ; inv(M_Dynamic_Mats.M)*(M_Dynamic_Mats.F-M_Dynamic_Mats.C*Xdot-M_Dynamic_Mats.G)] ;

end
