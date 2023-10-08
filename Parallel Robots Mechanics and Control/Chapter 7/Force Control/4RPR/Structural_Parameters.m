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
rad2deg = 180/pi;

%% Attachment Points Coordinates

SP.n   = 4 ;   % Number Of limbs 
SP.RA  = 900 ;   % the Ai's circle radius (m)
SP.RB  = 10  ;   % the Bi's circle radius (m)
SP.Ath = [-135; -45; 45; 135] * deg2rad ; % Angle between Fixed Attachment Points and X axis of Fixed Coordinate System
SP.Bth = [-45; -135; 135; 45] * deg2rad ; % Angle between Moving Attachment Points and x axis of Moving Coordinate System
SP.A   = zeros(3,4) ; % Fixed Attachment Points Positions in Fixed Coordinate System (m)
SP.B_B = zeros(3,4) ; % Moving Attachment Points Positions in Moving Coordinate System (m)
for i = 1:SP.n
  SP.A(:,i) = [SP.RA*cos(SP.Ath(i)); SP.RA*sin(SP.Ath(i)); 0] ;
  SP.B_B(:,i) = [SP.RB*cos(SP.Bth(i)); SP.RB*sin(SP.Bth(i)); 0] ;
end

%% Mass, Density and Inertia

SP.m = 2500 ;  % the Mass of the E.F. (Kg)
SP.Im = 3.5e5; % the Inertia Matrix of the E.F. relative to Moving Coordinate 
               % System about z axis(Kg.m^2)
SP.rho = 0.215; % The limb's cable density (Kg/m)
SP.pert=  0.50; % Parameter perturbation %10

% NLM=1;


%% Desired Trajectory 

%   The desired motion trajectory initial points
%     Time    x      y     \phi
% SP.xd=[
%         0     0      0        0*deg2rad
%         100   100    100     -45*deg2rad
%         200   100    100     -45*deg2rad
%         300   0      0        0*deg2rad
%         400   0      0        0*deg2rad
%         500   0      0        0*deg2rad
%         600   0      0        0*deg2rad
%         10000 0      0        0*deg2rad
%         ];
%   The desired force trajectory initial points
%     Time    Fx     Fy     \tau_\phi
SP.xd=[
        0     0      0       0
        100   100    100     0
        200   100    100     0
        300   0      0       0
        400   0      0       0
        500   0      0       0
        600   0      0       0
        10000 0      0       0
        ];  
%% Noise Amplitude
SP.noise_x  = 5e-4*[100;100;45*deg2rad]*0;  % Position measurement noise
SP.noise_dx = 5e-4*[2;2;0.02]*0;            % Velocity measurement noise
SP.noise_F  = 1e-1*[1;1;0]*0;               % Force    measurement noise

%% External Disturbance
%   The desired trajectory initial points
%     Time    F_dx   F_dy   \tau_d
Fout=[
     0       0         0        0
    75       1         1        1
    75.001   1         1        1
    150      1         1        1
    150.001  1         1        1
    200      1         1        1
  ];
SP.time=Fout(:,1);   
SP.Fd=Fout(:,2:4)*0; %*1e3;    % no disturbance
%
%  Specify it maually or load experiments:
%
                        %
% load experiment.mat     % Experimental Data 
%                         % Data structure name: Fout
% SP.time=Fout(:,1)/2;    % real time is scaled   
% SP.Fd=Fout(:,2:4)/5*0;    % real data is scaled       
% SP.Fd(:,3)= sqrt(Fout(:,2).^2 + Fout(:,3).^2)*0;    
    

%% Simulation Parameters
SP.ts = 0 ; SP.tf = 100 ;
SP.tspan=[SP.ts SP.tf];

SP.X0 = [0 ; 0 ; 0*pi/180] ;
SP.Xdot0 = [0 ; 0 ; 0] ;
SP.F0 = [0; 0; 0];

%% Environment Parameter

SP.g = [0 ; 0 ; -9.81] ; % Vector of Gravity Acceleration in the Fixed Coordinate System (m/s^2)
    
%  A linear surface passes throug a point with a slope

SP.xp=[75; 75];       % One point on the surface
SP.sl=-1;           % The slope of the surface

SP.Ke = 1e1*diag([1 1 0]);   % Environment stiffness coefficient
SP.Ce = 1e1*diag([1 1 0]);   % Environment damping coefficient


%% Controller Parameters
    
SP.Kp=20*diag([1 1 0]); % The desired impedance stiffness matrix
SP.Kd=5*diag([1 1 0]); % The desired impedance damping matrix
SP.Kf=0.05*diag([1 1 0]); % The desired impedance Mass matrix
SP.Ki=0.1*diag([1 1 0]); % The desired impedance Mass matrix

%% Redundancy Resolution Parameters

SP.Fmax    = 5000*[1; 1; 1; 1];   %Actuator upper limit (N)   
SP.Fmin    =  10*[1; 1; 1; 1];    %Actuator lower limit (N)  
SP.JFP_init= zeros(SP.n,1);       %Iitial guess for Redundancy Resolution

end