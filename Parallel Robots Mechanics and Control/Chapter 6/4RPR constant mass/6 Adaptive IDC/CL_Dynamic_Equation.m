%% 
%   Parallel Robots: Mechanics and Control
%   Copyright Hamid D. Taghirad 2013
%
%   This program generates the closed-loop dynamic formulation
%   of the planar cable manipulator
%
function [cost] = CL_Dynamic_Equation(t,X_s,SP)

%disp(t)             % Too keep track of integration time
X    = X_s(1:3) ;
Xdot = X_s(4:6) ;
th   = X_s(7:9);

[xd,dxd,d2xd]=TP_cubic_s(t, SP);
e=-[xd-X;dxd-Xdot];

[KC]             = Kinematic_Configuration(X,Xdot,SP) ;
[P_Dynamic_Mats] = Parts_Dynamic_Matrixes(SP,KC) ;
[M_Dynamic_Mats] = Manipulator_Dynamic_Matrixes(SP,P_Dynamic_Mats) ;
[M_Dynamic_Mats] = AIDC_Control(t,X_s,SP,KC,M_Dynamic_Mats,P_Dynamic_Mats) ;

cost = [Xdot ; 
    inv(M_Dynamic_Mats.M)*(M_Dynamic_Mats.F-M_Dynamic_Mats.C*Xdot-M_Dynamic_Mats.G);
    SP.Gamma_1 * M_Dynamic_Mats.Phi' * SP.MB' *SP.MP * e] ;

end
