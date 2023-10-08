%% 
%   Parallel Robots: Mechanics and Control
%   Copyright Hamid D. Taghirad 2013
%
%   This program generates the kinematic configuration
%   of the Stewart-Gough platform
%
function [Kinematic_Conf] = Kinematic_Configuration(X,Xdot,Struct_Param)
%% E.F. Pose

Kinematic_Conf.x = X(1:3) ; % Position Vector of the Origin of the Moving Coordinate System relative to Fixed Coordinate System (m)
Kinematic_Conf.phi = X(4:6) ; % Column Matrix of Euler angels; Orientation of E.F. (rad)
alpha = Kinematic_Conf.phi(1) ; % Euler Angle alpha (rad)
beta = Kinematic_Conf.phi(2) ; % Euler Angle beta (rad)
gamma = Kinematic_Conf.phi(3) ; % Euler Angle gamma (rad)
Kinematic_Conf.xdot = Xdot(1:3) ; % Velocity Vector of the Origin of the Moving Coordinate System relative to Fixed Coordinate System (m/s)
Kinematic_Conf.phidot = Xdot(4:6) ; % Column Matrix of the rate of the change in Euler angels; (m/s)
%alphadot = phidot(1) ; % (rad/s)
betadot = Kinematic_Conf.phidot(2) ; % (rad/s)
gammadot = Kinematic_Conf.phidot(3) ; % (rad/s)

%% Orientation Matrixes

Kinematic_Conf.R = [cos(beta)*cos(gamma) cos(gamma)*sin(alpha)*sin(beta)-cos(alpha)*sin(gamma) cos(alpha)*cos(gamma)*sin(beta)+sin(alpha)*sin(gamma)
     cos(beta)*sin(gamma) cos(alpha)*cos(gamma)+sin(alpha)*sin(beta)*sin(gamma) -cos(gamma)*sin(alpha)+cos(alpha)*sin(beta)*sin(gamma)
      -sin(beta)                            cos(beta)*sin(alpha)                                      cos(alpha)*cos(beta)            ]; % Rotation Matrix Form B to A
  
Kinematic_Conf.E = [cos(beta)*cos(gamma)  -sin(gamma)   0
                    cos(beta)*sin(gamma)   cos(gamma)   0
                    -sin(beta)            0        1  ]; 

Kinematic_Conf.Edot = [-betadot*sin(beta)*cos(gamma)-gammadot*cos(beta)*sin(gamma) -gammadot*cos(gamma) 0
                       -betadot*sin(beta)*sin(gamma)+gammadot*cos(beta)*cos(gamma) -gammadot*sin(gamma) 0
                            -betadot*cos(beta)                                            0             0] ;
         
%% E.F. Kinematics         

Kinematic_Conf.Omega_EF = Kinematic_Conf.E * Kinematic_Conf.phidot ; % Angular Velocity Vector of E.F. (rad/s)
Kinematic_Conf.Omega_EF_x = [ 0                           -Kinematic_Conf.Omega_EF(3)   Kinematic_Conf.Omega_EF(2)
                              Kinematic_Conf.Omega_EF(3)        0                       -Kinematic_Conf.Omega_EF(1)
                              -Kinematic_Conf.Omega_EF(2)  Kinematic_Conf.Omega_EF(1)            0                 ] ; % Skew-Symmetric Matrix Corresponding to Omega_EF (rad/s)
Kinematic_Conf.B = zeros(3,Struct_Param.n) ; % Position of Moving Attachment Points in Fixed Coordinate System (m)
Kinematic_Conf.B_x = zeros(3,3,Struct_Param.n) ; % Skew-Symmetric Matrix Corresponding to B (m)
Kinematic_Conf.I_EF = Kinematic_Conf.R*Struct_Param.I_EF*Kinematic_Conf.R' ;

%% Limbs Kinematics

Kinematic_Conf.l = zeros(1,Struct_Param.n) ; % Length of each Limb (m)
Kinematic_Conf.c2 = zeros(1,Struct_Param.n) ;
Kinematic_Conf.s = zeros(3,Struct_Param.n) ; % Unit Vector along each Limb 
Kinematic_Conf.s_x = zeros(3,3,Struct_Param.n) ; % Skew-Symmetric Matrix Corresponding to s
Kinematic_Conf.x_Bdot = zeros(3,Struct_Param.n) ; % velocity of Moving Attachment Points B_i (m/s)
Kinematic_Conf.ldot = zeros(1,Struct_Param.n) ; % Rate of change in each leg's length in workspace (m/s)
Kinematic_Conf.Omega_L = zeros(3,Struct_Param.n) ; % Angular Velocity Vector of each limb (rad/s)


for i = 1:Struct_Param.n 
    Kinematic_Conf.B(:,i) = Kinematic_Conf.R*Struct_Param.B_B(:,i) ; 
    Kinematic_Conf.B_x(:,:,i) = [ 0           -Kinematic_Conf.B(3,i)    Kinematic_Conf.B(2,i)
                   Kinematic_Conf.B(3,i)                    0           -Kinematic_Conf.B(1,i)
                   -Kinematic_Conf.B(2,i)     Kinematic_Conf.B(1,i)        0   ] ; 
    Kinematic_Conf.l(:,i) = norm(Kinematic_Conf.x+Kinematic_Conf.B(:,i)-Struct_Param.A(:,i)) ;
    Kinematic_Conf.s(:,i) = (Kinematic_Conf.x+Kinematic_Conf.B(:,i)-Struct_Param.A(:,i))/Kinematic_Conf.l(:,i) ;
    Kinematic_Conf.s_x(:,:,i) = [ 0           -Kinematic_Conf.s(3,i)          Kinematic_Conf.s(2,i)
                   Kinematic_Conf.s(3,i)        0              -Kinematic_Conf.s(1,i)
                   -Kinematic_Conf.s(2,i)     Kinematic_Conf.s(1,i)             0   ] ;      
    Kinematic_Conf.x_Bdot(:,i) = Kinematic_Conf.xdot+ Kinematic_Conf.Omega_EF_x * Kinematic_Conf.B(:,i) ;
    Kinematic_Conf.ldot(:,i) = Kinematic_Conf.x_Bdot(:,i)' * Kinematic_Conf.s(:,i) ;    
    Kinematic_Conf.Omega_L(:,i) = (1/Kinematic_Conf.l(:,i))*Kinematic_Conf.s_x(:,:,i)*Kinematic_Conf.x_Bdot(:,i) ;
    Kinematic_Conf.c2(:,i) = Kinematic_Conf.l(:,i) - Struct_Param.cp2(:,i) ;
    Kinematic_Conf.mc(:,i) = (1/Kinematic_Conf.l(:,i)^2)*(Struct_Param.m1(:,i)*Struct_Param.c1(:,i)^2+Struct_Param.m2(:,i)*Kinematic_Conf.c2(:,i)^2) ;
    Kinematic_Conf.mg(:,i) = (1/Kinematic_Conf.l(:,i))*(Struct_Param.m1(:,i)*Struct_Param.c1(:,i)+Struct_Param.m2(:,i)*Kinematic_Conf.c2(:,i)) ;
    Kinematic_Conf.mco(:,i) = (1/Kinematic_Conf.l(:,i))*Struct_Param.m2(:,i)*Kinematic_Conf.c2(:,i)-(1/Kinematic_Conf.l(:,i)^2)*Struct_Param.Ixx(:,i)-Kinematic_Conf.mc(:,i) ;
end

end