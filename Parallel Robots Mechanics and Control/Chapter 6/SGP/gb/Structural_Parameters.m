%% 
%   Parallel Robots: Mechanics and Control
%   Copyright Hamid D. Taghirad 2013
%
%   This program generates all the requires parameters of the 
%   programs in a structure.
%
function [Struct_Param] = Structural_Parameters()
%% 
deg2rad = pi/180 ;

%% Attachment Points Coordinates

Struct_Param.n = 6 ;   % Number Of limbs 
Struct_Param.RA = 2 ;   % the Ai's circle radius (m)
Struct_Param.RB = 1 ;   % the Bi's circle radius (m)
Struct_Param.Ath = [0; 60; 120; 180; 240; 300] * deg2rad ; % Angle between Fixed Attachment Points and X axis of Fixed Coordinate System
Struct_Param.Bth = [30; 30; 150; 150; 270; 270] * deg2rad ; % Angle between Moving Attachment Points and x axis of Moving Coordinate System
Struct_Param.A = zeros(3,6) ; % Fixed Attachment Points Positions in Fixed Coordinate System (m)
Struct_Param.B_B = zeros(3,6) ; % Moving Attachment Points Positions in Moving Coordinate System (m)
for i = 1:6
  Struct_Param.A(:,i) = [Struct_Param.RA*cos(Struct_Param.Ath(i)); Struct_Param.RA*sin(Struct_Param.Ath(i)); 0] ;
  Struct_Param.B_B(:,i) = [Struct_Param.RB*cos(Struct_Param.Bth(i)); Struct_Param.RB*sin(Struct_Param.Bth(i)); 0] ;
end

%% Mass, Density and Inertia

Struct_Param.m_EF = 1150 ;  % the Mass of the E.F. (Kg)
Struct_Param.I_EF = diag([570,285,285]) ; % the Inertia Matrix of the E.F. relative to Moving Coordinate System (Kg.m^2)

for i = 1:Struct_Param.n
    Struct_Param.c1(:,i) = 0.75;  % The center of Mass of ith Cylinder (m)
    Struct_Param.m1(:,i) = 85;    % Mass of ith Cylinder (Kg)
    Struct_Param.I1xx(:,i) = 16;   % Moment of inertia of ith symmetric Cylinder around moving x axis(Kg.m^2)
    Struct_Param.cp2(:,i) = 0.75;  % The center of Mass of ith Piston (m)
    Struct_Param.m2(:,i) = 22;    % Mass of ith Piston (Kg)
    Struct_Param.I2xx(:,i) = 4.1;  % Moment of inertia of ith symmetric Piston around moving x axis (Kg.m^2)
    Struct_Param.Ixx(:,i) = Struct_Param.I2xx(:,i) + Struct_Param.I1xx(:,i) ; % Moment of inertia of ith symmetric Leg around moving x axis (Kg.m^2)
end

%% Enviroment Factors

Struct_Param.g = [0 ; 0 ; -9.8] ; % Vector of  Gravity Acceleration in the Fixed Coordinate System (m/s^2)

%% Desired Trajectory and Controller Parameters

%   The desired trajectory initial points
%     Time    x      y     z    s_x   s_y   s_z  \theta
Struct_Param.xd=[
        0     0      0     1     1     1     1    0*deg2rad
        1     0.25   0.5   0.75  1     2     3    5*sqrt(14)*deg2rad
        2     0      0     1     1     1     1    0*deg2rad
        3     0      0     1     1     1     1    0*deg2rad
        4     0      0     0     1     1     1    0*deg2rad
        5     0      0     0     1     1     1    0*deg2rad
        6     0      0     0     1     1     1    0*deg2rad
        10    0      0     0     1     1     1    0*deg2rad
        ];
    
Struct_Param.KP=1e6*diag([1 1 1 10 10 10]); % The proportional controller gain KP
Struct_Param.KV=1e5*diag([1 1 1 1 1 1]); % The derivative controller gain KV
Struct_Param.pert=  1.0; % Parameter perturbation %0

%% Noise Amplitude
Struct_Param.noise_x  = 1e-4*[1;1;1;10*deg2rad;10*deg2rad;10*deg2rad]*0;  % no noise
Struct_Param.noise_dx = 1e-4*[1;1;1;1;1;1]*0;

%% External Disturbance
%   The desired trajectory initial points
%     Time    F_dx   F_dy     F_dz   \tau_x   \tau_y   \tau_z
Fout=[
     0        0       0       0       0        0       0
     0.50     1       1       1       0        0       0
     0.5001   1       1       1       0        0       0
     1.0      1       1       1       1       1      1
     1.001    1       1       1       1       1      1
     1.50     1       1       1       1       1      1
     1.5001   1       1       1       1       1      1
     10       1       1       1       1       1      1
    ];
Struct_Param.time=Fout(:,1);   
Struct_Param.Fd=Fout(:,2:7)*0;  %1e4;    % 50KN disturbance!
%% Determine Time Struct_Paraman
Struct_Param.ts = 0 ;
Struct_Param.tf = 2 ;
Struct_Param.tStruct_Paraman=[Struct_Param.ts Struct_Param.tf];

%% initial values
Struct_Param.X0 = [0 ; 0 ; +1 ; 0 ; 0 ; 0] ;
Struct_Param.Xdot0 = [0 ; 0 ; 0 ; 0 ; 0 ; 0] ;
   
%% This ends the structral parameters

end