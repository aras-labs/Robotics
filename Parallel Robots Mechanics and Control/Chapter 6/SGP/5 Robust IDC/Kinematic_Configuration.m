%% 
%   Parallel Robots: Mechanics and Control
%   Copyright Hamid D. Taghirad 2013
%
%   This program generates the kinematic configuration
%   of the Stewart-Gough platform
%
function [KC] = KC(X,Xdot,SP)
%% E.F. Pose

KC.x        = X(1:3) ;    % Position Vector of the Origin of the Moving Coordinate System relative to Fixed Coordinate System (m)
KC.theta    = X(4:6) ;    % Column Matrix of screw coordinates; Orientation of E.F. (rad)
KC.xdot     = Xdot(1:3) ; % Velocity Vector of the Origin of the Moving Coordinate System relative to Fixed Coordinate System (m/s)
KC.Omega_EF = Xdot(4:6) ; % Column Matrix of angular velocity (rad/s)

%% Orientation Matrixes
theta=norm(KC.theta);   % The equivalent orientation angle;
sth=sin(theta);cth=cos(theta);vth=1-cth;    %some short hand notation
if theta < eps,
    s=[1;0;0];
else
    s=KC.theta/theta;
end
sx = s(1) ; % screw coordinate components
sy = s(2) ; % screw coordinate components
sz = s(3) ; % screw coordinate components

% Rotation matrix by screw representation
KC.R = [sx^2*vth+cth        sx*sy*vth-sz*sth        sx*sz*vth+sy*sth
        sy*sx*vth+sz*sth    sy^2*vth+cth            sy*sz*vth-sx*sth
        sz*sx*vth-sy*sth    sz*sy*vth+sx*sth        sz^2*vth+cth
        ]; % Rotation Matrix Form B to A
  

%% E.F. Kinematics         

KC.Omega_EF_x = [ 0          -KC.Omega_EF(3)   KC.Omega_EF(2)
            KC.Omega_EF(3)        0           -KC.Omega_EF(1)
           -KC.Omega_EF(2)  KC.Omega_EF(1)            0] ; % Skew-Symmetric Matrix Corresponding to Omega_EF (rad/s)
KC.B = zeros(3,SP.n) ;     % Position of Moving Attachment Points in Fixed Coordinate System (m)
KC.B_x = zeros(3,3,SP.n) ; % Skew-Symmetric Matrix Corresponding to B (m)
KC.I_EF = KC.R*SP.I_EF*KC.R' ;

%% Limbs Kinematics

KC.l       = zeros(1,SP.n) ;  % Length of each Limb (m)
KC.c2      = zeros(1,SP.n) ;
KC.s       = zeros(3,SP.n) ;  % Unit Vector along each Limb 
KC.s_x     = zeros(3,3,SP.n); % Skew-Symmetric Matrix Corresponding to s
KC.x_Bdot  = zeros(3,SP.n) ;  % velocity of Moving Attachment Points B_i (m/s)
KC.ldot    = zeros(1,SP.n) ;  % Rate of change in each leg's length in workspace (m/s)
KC.Omega_L = zeros(3,SP.n) ;  % Angular Velocity Vector of each limb (rad/s)


for i = 1:SP.n 
    KC.B(:,i) = KC.R*SP.B_B(:,i) ; 
    KC.B_x(:,:,i) = [ 0          -KC.B(3,i)    KC.B(2,i)
                   KC.B(3,i)       0           -KC.B(1,i)
                  -KC.B(2,i)     KC.B(1,i)        0   ] ; 
    KC.l(:,i) = norm(KC.x+KC.B(:,i)-SP.A(:,i)) ;
    KC.s(:,i) = (KC.x+KC.B(:,i)-SP.A(:,i))/KC.l(:,i) ;
    KC.s_x(:,:,i) = [ 0           -KC.s(3,i)          KC.s(2,i)
                   KC.s(3,i)        0              -KC.s(1,i)
                  -KC.s(2,i)     KC.s(1,i)             0   ] ;      
    KC.x_Bdot(:,i) = KC.xdot+ KC.Omega_EF_x * KC.B(:,i) ;
    KC.ldot(:,i) = KC.x_Bdot(:,i)' * KC.s(:,i) ;    
    KC.Omega_L(:,i) = (1/KC.l(:,i))*KC.s_x(:,:,i)*KC.x_Bdot(:,i) ;
    KC.c2(:,i) = KC.l(:,i) - SP.cp2(:,i) ;
    KC.mc(:,i) = (1/KC.l(:,i)^2)*(SP.m1(:,i)*SP.c1(:,i)^2+...
        SP.m2(:,i)*SP.cp2(:,i)^2) ;
    KC.mg(:,i) = (1/KC.l(:,i))*(SP.m1(:,i)*SP.c1(:,i)+SP.m2(:,i)*KC.c2(:,i)) ;
    KC.mco(:,i) = (1/KC.l(:,i))*SP.m2(:,i)*SP.cp2(:,i)-...
        (1/KC.l(:,i)^2)*SP.Ixx(:,i)-KC.mc(:,i) ;
end

end