%% 
%   Parallel Robots: Mechanics and Control
%   Copyright Hamid D. Taghirad 2013
%
%   This program generates the total dynamic matrices 
%   of the Stewart-Gough platform
%
function [M_Dynamic_Mats] = Manipulator_Dynamic_Matrixes(Struct_Param,P_Dynamic_Mats)

M_Dynamic_Mats.M = P_Dynamic_Mats.M_EF ; % M : Matrix M of the Manipulator
M_Dynamic_Mats.C = P_Dynamic_Mats.C_EF ; % C : Matrix C of the Manipulator
M_Dynamic_Mats.G = P_Dynamic_Mats.G_EF ; % G : Matrix G of the Manipulator

for i = 1:Struct_Param.n
    M_Dynamic_Mats.M = M_Dynamic_Mats.M + P_Dynamic_Mats.M_l(:,:,i) ; 
    M_Dynamic_Mats.C = M_Dynamic_Mats.C + P_Dynamic_Mats.C_l(:,:,i) ;
    M_Dynamic_Mats.G = M_Dynamic_Mats.G + P_Dynamic_Mats.G_l(:,:,i) ;
end
%
% Generate dynamics matrices in joint space 
%
M_Dynamic_Mats.Mq  = P_Dynamic_Mats.J_T * M_Dynamic_Mats.M * P_Dynamic_Mats.J_1;
M_Dynamic_Mats.Cq  = P_Dynamic_Mats.J_T * (M_Dynamic_Mats.C - M_Dynamic_Mats.M * ...
     P_Dynamic_Mats.J_1 * P_Dynamic_Mats.dJT')* P_Dynamic_Mats.J_1;
M_Dynamic_Mats.Gq  = P_Dynamic_Mats.J_T * M_Dynamic_Mats.G;
 
end
