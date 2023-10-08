%% 
%   Parallel Robots: Mechanics and Control
%   Copyright Hamid D. Taghirad 2013
%
%   This program generates the kinematic configuration
%   of the planar cable manipulator
%
function [KC] = Kinematic_Configuration(X,Xdot,SP)
%% E.F. Pose

KC.x = [X(1:2);0] ; % Position Vector of the Origin of the Moving Coordinate System relative to Fixed Coordinate System (m)
KC.phi = X(3) ; % Column Matrix of Euler angels; Orientation of E.F. (rad)
KC.xdot = [Xdot(1:2);0] ; % Velocity Vector of the Origin of the Moving Coordinate System relative to Fixed Coordinate System (m/s)
KC.phidot = Xdot(3) ; % Column Matrix of the rate of the change in Euler angels; (m/s)


%% Limbs Kinematics

KC.l     = zeros(1,SP.n) ; % Length of each Limb (m)
KC.s     = zeros(3,SP.n) ; % Unit Vector along each Limb 
KC.s_x   = zeros(3,3,SP.n) ; % Skew-Symmetric Matrix Corresponding to s
KC.B     = zeros(3,SP.n) ; % Position of Moving Attachment Points in Fixed Coordinate System (m)
% KC.B_x   = zeros(3,3,SP.n) ; % Skew-Symmetric Matrix Corresponding to B (m)
KC.E     = zeros(3,SP.n) ; % Position of Moving Attachment Points in moving Coordinate System (m)
KC.ldot  = zeros(1,SP.n) ; % Rate of change in each leg's length in workspace (m/s)
KC.omega = zeros(3,SP.n) ; % Angular Velocity Vector of each limb (rad/s)
KC.v_B   = zeros(3,SP.n) ; % Linear velocity of point Bi (m/s)
KC.J     = zeros(SP.n,3) ; % Robot Jacobian 
KC.Jalpha= zeros(SP.n,3) ; % Robot Jacobian for angular orientation alpha
 for i = 1:SP.n 
    KC.E(:,i)     = SP.RB*[cos(KC.phi+SP.Bth(i)); sin(KC.phi+SP.Bth(i)); 0];
    KC.B(:,i)     = KC.x - SP.A(:,i) + KC.E(:,i);
    KC.l(:,i)     = norm(KC.B(:,i)) ;                    %(3.10)
    KC.s(:,i)     = KC.B(:,i)/KC.l(:,i) ;
    KC.s_x(:,:,i) = [ 0           -KC.s(3,i)          KC.s(2,i)
                   KC.s(3,i)        0              -KC.s(1,i)
                   -KC.s(2,i)     KC.s(1,i)             0   ] ;
    KC.J(i,:) = [KC.s(1,i)  KC.s(2,i)  KC.E(1,i)*KC.s(2,i)-KC.E(2,i)*KC.s(1,i)];      % (4.54)
    KC.ldot(:,i)  = KC.J(i,:)*Xdot;                                                   % (4.53)    
    KC.Jalpha(i,:)= [-KC.s(2,i)  KC.s(1,i)  KC.E(1,i)*KC.s(1,i)+KC.E(2,i)*KC.s(2,i)]; % (4.58)
    KC.omega(:,i) = [0 ; 0; 1/KC.l(:,i)* KC.Jalpha(i,:)* Xdot];                       % (4.57)
    KC.v_B(:,i)   = KC.ldot(:,i)* KC.s(:,i) + KC.l(:,i)*cross(KC.omega(:,i),KC.s(:,i)); % similar to (5.304)
 end

end