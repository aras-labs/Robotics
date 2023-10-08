%% 
%   Parallel Robots: Mechanics and Control
%   Copyright Hamid D. Taghirad 2013
%
%   This program generates the dynamic matrices of the compontnts
%   of the Stewart-Gough platform
%
function [P_Dynamic_Mats] = Parts_Dynamic_Matrixes(SP,KC)
%% Dynamic Matrixes of the E.F.

P_Dynamic_Mats.M_EF = [SP.m_EF*eye(3,3) zeros(3,3)
                       zeros(3,3)    KC.I_EF] ; % Matrix M Of the E.F.
P_Dynamic_Mats.C_EF = [zeros(3,3)    zeros(3,3)
                       zeros(3,3)    KC.Omega_EF_x*KC.I_EF] ; % Matrix C Of the E.F.
P_Dynamic_Mats.G_EF = [-SP.m_EF*SP.g
                       zeros(3,1)] ; % Matrix G Of the E.F.

%% Dynamic Matrixes of the Cables

P_Dynamic_Mats.M_l = zeros(6,6,SP.n) ; % M_C(:,:,i) : Matrix M Of the Cable i
P_Dynamic_Mats.C_l = zeros(6,6,SP.n) ; % C_C(:,:,i) : Matrix C Of the Cable i
P_Dynamic_Mats.G_l = zeros(6,1,SP.n) ; % G_C(:,:,i) : Matrix G Of the Cable i
P_Dynamic_Mats.J = zeros(3,6,SP.n) ; % Coordinate Transformation Jacobian (diffrent from Robot Jacobian)
P_Dynamic_Mats.JT = zeros(6,SP.n) ;  % Robot Jacobian Transpose
P_Dynamic_Mats.Jdot = zeros(3,6,SP.n) ;

for i = 1:SP.n
    
    M_temp = SP.m2(:,i)*KC.s(:,i)*KC.s(:,i)'- ...
    (1/KC.l(:,i)^2)*SP.Ixx(:,i)*KC.s_x(:,:,i)^2- ...
    KC.mc(:,i)*KC.s_x(:,:,i)^2 ;
    C_temp = (((-2/KC.l(:,i))*KC.mco(:,i)*KC.ldot(:,i)) - ...
    (1/KC.l(:,i)^2)*SP.m2(:,i)*SP.cp2(:,i)*KC.s(:,i)*KC.x_Bdot(:,i)')*KC.s_x(:,:,i)^2 ;        
    G_temp = (KC.mg(:,i)*KC.s_x(:,:,i)^2 - ...
     SP.m2(:,i)*KC.s(:,i)*KC.s(:,i)')*SP.g ;
    
    P_Dynamic_Mats.JT(:,i) = [KC.s(:,i); ...
        KC.B_x(:,:,i)*KC.s(:,i)];
    P_Dynamic_Mats.J(:,:,i) = [eye(3,3) -KC.B_x(:,:,i)] ;
    P_Dynamic_Mats.Jdot(:,:,i) = ...
        [zeros(3,3) -KC.Omega_EF_x*KC.B_x(:,:,i)+...
     KC.B_x(:,:,i)*KC.Omega_EF_x] ;    
    
    P_Dynamic_Mats.M_l(:,:,i) = P_Dynamic_Mats.J(:,:,i)'*M_temp*P_Dynamic_Mats.J(:,:,i) ;
    P_Dynamic_Mats.C_l(:,:,i) = P_Dynamic_Mats.J(:,:,i)'*M_temp*P_Dynamic_Mats.Jdot(:,:,i)+ ...
    P_Dynamic_Mats.J(:,:,i)'*C_temp*P_Dynamic_Mats.J(:,:,i) ;
    P_Dynamic_Mats.G_l(:,:,i) = P_Dynamic_Mats.J(:,:,i)'*G_temp ;    
    
end
end