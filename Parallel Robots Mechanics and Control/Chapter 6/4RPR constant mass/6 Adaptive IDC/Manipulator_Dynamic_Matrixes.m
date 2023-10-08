%% 
%   Parallel Robots: Mechanics and Control
%   Copyright Hamid D. Taghirad 2013
%
%   This program generates the total dynamic matrices 
%   of the planar cable manipulator
%
function [M_Dynamic_Mats] = Manipulator_Dynamic_Matrixes(SP,P_Dynamic_Mats)

M_Dynamic_Mats.M = P_Dynamic_Mats.M_EF ; % M : Matrix M of the Manipulator
M_Dynamic_Mats.C = P_Dynamic_Mats.C_EF ; % C : Matrix C of the Manipulator
M_Dynamic_Mats.G = P_Dynamic_Mats.G_EF ; % G : Matrix G of the Manipulator

M_Dynamic_Mats.Mr = zeros(3,3) ;            % Mass Matrix without rho 
M_Dynamic_Mats.Cr = zeros(3,3) ;            % C Matrix without rho 
M_Dynamic_Mats.Gr = zeros(3,1) ;            % G Vector without rho 

for i = 1:SP.n
    M_Dynamic_Mats.M  = M_Dynamic_Mats.M  + P_Dynamic_Mats.M_l(:,:,i) ; 
    M_Dynamic_Mats.C  = M_Dynamic_Mats.C  + P_Dynamic_Mats.C_l(:,:,i) ;
    M_Dynamic_Mats.G  = M_Dynamic_Mats.G  + P_Dynamic_Mats.G_l(:,:,i) ;
    M_Dynamic_Mats.Mr = M_Dynamic_Mats.Mr + P_Dynamic_Mats.M_l(:,:,i)/SP.rho ; 
    M_Dynamic_Mats.Cr = M_Dynamic_Mats.Cr + P_Dynamic_Mats.C_l(:,:,i)/SP.rho ;
    M_Dynamic_Mats.Gr = M_Dynamic_Mats.Gr + P_Dynamic_Mats.G_l(:,:,i)/SP.rho ;
end

M_Dynamic_Mats.F = zeros(3,1) ;         % disturbance force
% interpolates disturbance inputs from its given the time history 
% %
% M_Dynamic_Mats.F(1,1)=interp1(SP.time,SP.Fd(:,1),t); % Experimental wind disturbance
% M_Dynamic_Mats.F(2,1)=interp1(SP.time,SP.Fd(:,2),t);
% M_Dynamic_Mats.F(3,1)=interp1(SP.time,SP.Fd(:,3),t);

end