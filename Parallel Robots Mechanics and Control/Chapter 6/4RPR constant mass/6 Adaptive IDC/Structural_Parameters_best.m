%% 
%   Parallel Robots: Mechanics and Control
%   Copyright Hamid D. Taghirad 2013
%
%   	This program generates all the requires parameters of the 
%       programs in a structure. Best parameters for simulation is 
%	saved in this file as a back up for the user.
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


%% Enviroment Factors

SP.g = [0 ; 0 ; -9.81] ; % Vector of Gravity Acceleration in the Fixed Coordinate System (m/s^2)

%% Desired Trajectory 

%   The desired trajectory initial points
%     Time    x      y     \phi
SP.xd=[
        0     0      0        0*deg2rad
        100   100    100     -45*deg2rad
        200   0      0        0*deg2rad
        300   0      0        0*deg2rad
        400   0      0        0*deg2rad
        500   0      0        0*deg2rad
        600   0      0        0*deg2rad
        10000 0      0        0*deg2rad
        ];
    
%% Noise Amplitude
SP.noise_x  = 5e-4*[100;100;45*deg2rad]*0;  % no noise
SP.noise_dx = 5e-4*[2;2;0.02]*0;

%% External Disturbance
%   The desired trajectory initial points
%     Time    F_dx   F_dy   \tau_d
Fout=[
     0       0         0       0
    50       0         0       0
    50.001   1000      0       0
    100      1000      0       0
    100.001  1000      1000    0
    150      1000      1000    0
    150.001  1000      1000    1000
    200      1000      1000    1000
  ];
SP.time=Fout(:,1);   
SP.Fd=Fout(:,2:4)*0;    % no disturbance
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
SP.ts = 0 ; SP.tf = 200 ;
SP.tspan=[SP.ts SP.tf];

SP.X0 = [0 ; 0 ; 0*pi/180] ;
SP.Xdot0 = [0 ; 0 ; 0] ;
SP.th0 = [SP.m ; SP.Im ; SP.rho]*SP.pert;


%% Controller Parameters
    
SP.KP=diag([1 1 1]); % The proportional controller gain KP
SP.KV=2.8284*diag([1 1 1]); % The  derivative  controller gain KV

% Adaptive control Parameters

SP.MA=[zeros(3,3) eye(3,3); -SP.KP -SP.KV];  % Book eq. (6.27)
SP.MB=[zeros(3,3); eye(3,3)];                % Book eq. (6.27)   
SP.MQ=eye(6,6);                              % Book eq. (6.28)
SP.MP=lyap(SP.MA,SP.MQ);                     % Book eq. (6.28)

% Adaptation Gain
SP.Gamma_1 = 1e3*diag([SP.m^2*100 SP.Im^2*5000 -SP.rho^2*100]);

    
%% Redundancy Resolution Parameters

SP.Fmax    = 5000*[1; 1; 1; 1];   %Actuator upper limit (N)   
SP.Fmin    =  10*[1; 1; 1; 1];    %Actuator lower limit (N)  
SP.JFP_init= zeros(SP.n,1);       %Iitial guess for Redundancy Resolution

end