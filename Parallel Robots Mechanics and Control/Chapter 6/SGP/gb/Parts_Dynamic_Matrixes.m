%% 
%   Parallel Robots: Mechanics and Control
%   Copyright Hamid D. Taghirad 2013
%
%   This program generates the dynamic matrices of the compontnts
%   of the Stewart-Gough platform
%
function [P_Dynamic_Mats] = Parts_Dynamic_Matrixes(Struct_Param,Kinematic_Conf)
%% Dynamic Matrixes of the E.F.

P_Dynamic_Mats.M_EF = [Struct_Param.m_EF*eye(3,3) zeros(3,3)
                       zeros(3,3)    Kinematic_Conf.E'*Kinematic_Conf.I_EF*Kinematic_Conf.E ] ; % Matrix M Of the E.F.
P_Dynamic_Mats.C_EF = [zeros(3,3)    zeros(3,3)
                       zeros(3,3)    Kinematic_Conf.E'*Kinematic_Conf.I_EF*Kinematic_Conf.Edot+Kinematic_Conf.E'*Kinematic_Conf.Omega_EF_x*Kinematic_Conf.I_EF*Kinematic_Conf.E] ; % Matrix C Of the E.F.
P_Dynamic_Mats.G_EF = [-Struct_Param.m_EF*Struct_Param.g
                       zeros(3,1)] ; % Matrix G Of the E.F.

%% Dynamic Matrixes of the Cables

P_Dynamic_Mats.M_l = zeros(6,6,Struct_Param.n) ; % M_C(:,:,i) : Matrix M Of the Cable i
P_Dynamic_Mats.C_l = zeros(6,6,Struct_Param.n) ; % C_C(:,:,i) : Matrix C Of the Cable i
P_Dynamic_Mats.G_l = zeros(6,1,Struct_Param.n) ; % G_C(:,:,i) : Matrix G Of the Cable i
P_Dynamic_Mats.J = zeros(3,6,Struct_Param.n) ; % Coordinate Transformation Jacobian (diffrent from Robot Jacobian)
P_Dynamic_Mats.JT = zeros(6,Struct_Param.n) ;  % Robot Jacobian Transpose
P_Dynamic_Mats.Jdot = zeros(3,6,Struct_Param.n) ;

for i = 1:Struct_Param.n
    
    M_temp = Struct_Param.m2(:,i)*Kinematic_Conf.s(:,i)*Kinematic_Conf.s(:,i)'-(1/Kinematic_Conf.l(:,i)^2)*Struct_Param.Ixx(:,i)*Kinematic_Conf.s_x(:,:,i)^2-Kinematic_Conf.mc(:,i)*Kinematic_Conf.s_x(:,:,i)^2 ;
    C_temp = (((-2/Kinematic_Conf.l(:,i))*Kinematic_Conf.mco(:,i)*Kinematic_Conf.ldot(:,i))*eye(3)-(1/Kinematic_Conf.l(:,i)^2)*Struct_Param.m2(:,i)*Struct_Param.cp2(:,i)*Kinematic_Conf.s(:,i)*Kinematic_Conf.x_Bdot(:,i)')*Kinematic_Conf.s_x(:,:,i)^2 ;        
    G_temp = (Kinematic_Conf.mg(:,i)*Kinematic_Conf.s_x(:,:,i)^2 - Struct_Param.m2(:,i)*Kinematic_Conf.s(:,i)*Kinematic_Conf.s(:,i)')*Struct_Param.g ;
    
    P_Dynamic_Mats.JT(:,i) = [Kinematic_Conf.s(:,i); Kinematic_Conf.B_x(:,:,i)*Kinematic_Conf.s(:,i)];
    P_Dynamic_Mats.J(:,:,i) = [eye(3,3) -Kinematic_Conf.B_x(:,:,i)*Kinematic_Conf.E] ;
    P_Dynamic_Mats.Jdot(:,:,i) = [zeros(3,3) -Kinematic_Conf.Omega_EF_x*Kinematic_Conf.B_x(:,:,i)*Kinematic_Conf.E+Kinematic_Conf.B_x(:,:,i)*Kinematic_Conf.Omega_EF_x*Kinematic_Conf.E+Kinematic_Conf.B_x(:,:,i)*Kinematic_Conf.Edot] ;    
    
    P_Dynamic_Mats.M_l(:,:,i) = P_Dynamic_Mats.J(:,:,i)'*M_temp*P_Dynamic_Mats.J(:,:,i) ;
    P_Dynamic_Mats.C_l(:,:,i) = P_Dynamic_Mats.J(:,:,i)'*M_temp*P_Dynamic_Mats.Jdot(:,:,i)+P_Dynamic_Mats.J(:,:,i)'*C_temp*P_Dynamic_Mats.J(:,:,i) ;
    P_Dynamic_Mats.G_l(:,:,i) = P_Dynamic_Mats.J(:,:,i)'*G_temp ;    
    
end
end