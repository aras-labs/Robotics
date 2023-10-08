%% 
%   Parallel Robots: Mechanics and Control
%   Copyright Hamid D. Taghirad 2013
%
%   This program generates all the requires parameters of the 
%   programs in a structure.
%
function [SP] = Structural_Parameters()
%% 
deg2rad = pi/180 ;

%% Attachment Points Coordinates

SP.n = 6 ;    % Number Of limbs 
SP.RA = 2 ;   % the Ai's circle radius (m)
SP.RB = 1 ;   % the Bi's circle radius (m)
SP.Ath = [0; 60; 120; 180; 240; 300] * deg2rad ; % Angle between Fixed Attachment Points and X axis of Fixed Coordinate System
SP.Bth = [30; 30; 150; 150; 270; 270] * deg2rad ; % Angle between Moving Attachment Points and x axis of Moving Coordinate System
SP.A = zeros(3,6) ; % Fixed Attachment Points Positions in Fixed Coordinate System (m)
SP.B_B = zeros(3,6) ; % Moving Attachment Points Positions in Moving Coordinate System (m)
for i = 1:6
  SP.A(:,i) = [SP.RA*cos(SP.Ath(i)); SP.RA*sin(SP.Ath(i)); 0] ;
  SP.B_B(:,i) = [SP.RB*cos(SP.Bth(i)); SP.RB*sin(SP.Bth(i)); 0] ;
end

%% Mass, Density and Inertia

SP.m_EF = 1150 ;  % the Mass of the E.F. (Kg)
SP.I_EF = diag([570,285,285]) ; % the Inertia Matrix of the E.F. relative to Moving Coordinate System (Kg.m^2)

for i = 1:SP.n
    SP.c1(:,i) = 0.75;      % The center of Mass of ith Cylinder (m)
    SP.m1(:,i) = 85;        % Mass of ith Cylinder (Kg)
    SP.I1xx(:,i) = 16 ;     % Moment of inertia of ith symmetric Cylinder around moving x axis(Kg.m^2)
    SP.cp2(:,i) = 0.75;     % The center of Mass of ith Piston (m)
    SP.m2(:,i) = 22;        % Mass of ith Piston (Kg)
    SP.I2xx(:,i) = 4.1;     % Moment of inertia of ith symmetric Piston around moving x axis (Kg.m^2)
    SP.Ixx(:,i) = SP.I2xx(:,i) + ...
    SP.I1xx(:,i) ;          % Moment of inertia of ith symmetric Leg around moving x axis (Kg.m^2)
end

%% Enviroment Factors

SP.g = [0 ; 0 ; -9.81] ; % Vector of  Gravity Acceleration in the Fixed Coordinate System (m/s^2)

%% Desired Trajectory and Controller Parameters

%   The desired trajectory initial points
%     Time    x      y     z    s_x   s_y   s_z  \theta
sq14=sqrt(14);
SP.xd=[
        0       0       0       1        1       1       1       0*deg2rad
        1.0    .25      .25    1.25      1       2       3       5*sq14*deg2rad
        2.0    .25      .25    1.25      1       2       3       5*sq14*deg2rad
        10.0   .25      .25    1.25      1       2       3       5*sq14*deg2rad
        10       0       0       1       1       1       1       0*deg2rad
        40       0       0       1       0       0       1       0*deg2rad
        50       0       0       1       0       0       1       0*deg2rad
        60       0       0       1       0       0       1       0*deg2rad
        10       0       0       1       0       0       1       0*deg2rad
        ];
    
%% Environment Parameters
    
%  A linear plane passes throug a point and has a normal to the plane
%  The normal slope muust not contain any zero component

SP.xp=[0.2;0.2;1.2];            % One point on the plane
SP.nl=[1;1;1];                     % The normal vector to the plane
SP.nl = SP.nl ./ norm(SP.nl);      % Normalize the slope

SP.Ke = 1e5*diag([1 1 1 0 0 0]);   % Environment stiffness coefficient
SP.Ce = 1e4*diag([1 1 1 0 0 0]);   % Environment damping coefficient


%% Controller Parameters
    
SP.Kd=1e4*diag([1 1 1 1 1 1]); % The desired impedance stiffness matrix
SP.Cd=2e3*diag([1 1 1 1 1 1]); % The desired impedance damping matrix
SP.Md=1e2*diag([1 1 1 1 1 1]); % The desired impedance Mass matrix
SP.pert=  0.9 ; % Parameter perturbation %10

%% Noise Amplitude
SP.noise_x  = 1e-3*[1;1;1;1*deg2rad;1*deg2rad;1*deg2rad]*0;  % no noise
SP.noise_dx = 1e-3*[1;1;1;1;1;1]*0;
SP.noise_F  = 1e+0*[1;1;0;1;1;1]*0;               % Force    measurement noise

%% External Disturbance
%   The desired trajectory initial points
%     Time    F_dx   F_dy     F_dz   \tau_x   \tau_y   \tau_z
Fout=[
     0        0       0       0       0        0       0
     0.50     0       0       0       0        0       0
     0.5001   1       1       1       0        0       0
     2.0      1       1       1       0        0       0
     2.001    0       0       0       0        0       0
     2.50     0       0       0       0        0       0
     2.5001   0       0       0       0        0       0
     10       0       0       0       0        0       0
    ];
SP.time=Fout(:,1);   
SP.Fd=Fout(:,2:7)*1e4; %1e4;    % 10KN disturbance!
%% Determine Time Span
SP.ts = 0 ;
SP.tf = 2.0 ;
SP.tspan=[SP.ts SP.tf];

%% initial values
SP.X0 = [0 ; 0 ; +1 ; 0 ; 0 ; 0] ;
SP.Xdot0 = [0 ; 0 ; 0 ; 0 ; 0 ; 0] ;
   
%% This ends the structral parameters
end