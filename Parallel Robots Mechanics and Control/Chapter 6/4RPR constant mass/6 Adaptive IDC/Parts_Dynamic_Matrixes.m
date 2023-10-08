%% 
%   Parallel Robots: Mechanics and Control
%   Copyright Hamid D. Taghirad 2013
%
%   This program generates the dynamic matrices of the compontnts
%   of the planar cable manipulator
%
function [P_Dynamic_Mats] = Parts_Dynamic_Matrixes(SP,KC)

%% Dynamic Matrixes of the E.F.

P_Dynamic_Mats.M_EF = [SP.m  0 0; 0 SP.m 0; 0 0 SP.Im];   % Matrix M Of the% E.F.
P_Dynamic_Mats.C_EF = zeros(3,3) ;                        % Matrix C Of the E.F.
P_Dynamic_Mats.G_EF = -SP.m*[SP.g(1);SP.g(2);0];          % Matrix G Of the E.F.

%% Dynamic Matrixes of the Cables

P_Dynamic_Mats.M_l  = zeros(3,3,SP.n) ; % M_C(:,:,i) : Matrix M Of the Cable i
P_Dynamic_Mats.C_l  = zeros(3,3,SP.n) ; % C_C(:,:,i) : Matrix C Of the Cable i
P_Dynamic_Mats.G_l  = zeros(3,1,SP.n) ; % G_C(:,:,i) : Matrix G Of the Cable i
P_Dynamic_Mats.J    = zeros(3,3,SP.n) ; % Coordinate Transformation Jacobian (diffrent from Robot Jacobian)
P_Dynamic_Mats.Jdot = zeros(3,3,SP.n) ;

for i = 1:SP.n
    
    M_temp = SP.rho*KC.l(:,i)*(eye(3,3)+5/12 * KC.s_x(:,:,i)^2) ;                       % (5.313)
    C_temp = SP.rho*(KC.ldot(:,i)- 1/2*KC.s(:,i)*KC.v_B(:,i)'- ...
        5/8*KC.s(:,i)*KC.v_B(:,i)'*KC.s_x(:,:,i)^2+ 5/12*KC.ldot(:,i)*KC.s_x(:,:,i)^2); % (5.333)     
    G_temp = SP.rho*KC.l(:,i)*(1/2*KC.s_x(:,:,i)^2 - KC.s(:,i)*KC.s(:,i)')*SP.g ;       % (5.324)           
    
    P_Dynamic_Mats.J(:,:,i) = [[1;0;0] [0;1;0] cross([0;0;1],KC.E(:,i))] ;              % (5.343)
    P_Dynamic_Mats.Jdot(:,:,i) = [zeros(3,2) -KC.E(:,i)*KC.phidot] ;                    % (5.344)    
    
    P_Dynamic_Mats.M_l(:,:,i) = P_Dynamic_Mats.J(:,:,i)'*M_temp*P_Dynamic_Mats.J(:,:,i) ;
    P_Dynamic_Mats.C_l(:,:,i) = P_Dynamic_Mats.J(:,:,i)'*M_temp*P_Dynamic_Mats.Jdot(:,:,i)+ ...
        P_Dynamic_Mats.J(:,:,i)'*C_temp*P_Dynamic_Mats.J(:,:,i) ;
    P_Dynamic_Mats.G_l(:,:,i) = P_Dynamic_Mats.J(:,:,i)'*G_temp ;    
    
end
end