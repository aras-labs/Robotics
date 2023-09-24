%% 
% 	An Introduction to Robotics, 
%   By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
%   Copyright ARAS @2023
%
%   This program generates all the requires parameters of the 
%   programs in a structure.
%
function [SP] = Structural_Parameters()
%% 
deg2rad = pi/180 ; rad2deg = 180/pi;

%% Kinematics and Dynamics Parameters

SP.n   = 3;                 % No. of degrees of Freedom
SP.a   = [1 ; 0.7; 0.3];    %   Link lengths
SP.m = [10; 7; 3];          % the Mass of the E.F. (Kg)
SP.g = -9.81;               % Gravity Acceleration (m/s^2)
SP.pert = 0.8;                % (1-SP.pert)*100% parameter perterbation

%% Stiff Environment
%  A linear surface passes throug a point SP.xp with a slope Sp.sl

SP.xl = 1.1; SP.yl = 1.1;         

SP.sl=1;                    % The slope of the line

SP.Ke = 1e4*diag([1 1 0]);   % Environment stiffness coefficient
SP.Ce = 0e2*diag([1 1 0]);   % Environment damping coefficient

%% Desired Trajectory in task space

SP.phi = -atan(SP.sl);   % Normal to the flat object surface
x0 = SP.xl; y0= SP.yl;
xf = 0.5; yf= SP.yl + SP.sl*(xf-SP.xl); % Find the second point coordinates
SP.del=0.1; del=SP.del;

%   The desired trajectory initial points
%     Time    Fdx       Fdy       Fdz
SP.Fd=[
        0       0        0         0
%        0.5    -200     200        0
        3      -200     200        0
        4      -200     200        0
        6       0        0         0
        8       0        0         0
        10      0        0         0
        ];
SP.ts = 0 ; SP.tf = 4;
    
%% load Noise file
load snoise.mat;
SP.noise=qnoise;
SP.Isn=0;       % 0: no noise, 1: Is noise
clear qnoise

%% External Disturbance
%   The desired trajectory initial points
%     Time    \tau_1   \tau_2   \tau_3
taud=[
     0       0       0       0
    1        0       0       0
    1.001   -100   -100    -10
    10      -100   -100    -10
  ];
SP.time=taud(:,1);   
SP.taud=taud(:,2:4);    
SP.Isd=1;       % 0:no disturbance;   1:Is disturbance
%%
%  Actuator limitations
%
SP.tau_limit=[500; 500; 10];    % Torque limits
SP.Istau=1;      % 0:no saturation;   1:With Saturation

%% Simulation Parameters
SP.tspan=[SP.ts SP.tf];                       
SP.x0 =  [x0-SP.del/10; y0+SP.del/10; SP.phi];
SP.qd0 = [0 ; 0 ; 0];
SP.taui0 = [0 ; 0 ; 0];  % initial condition for taui
SP.taudi0 = [0 ; 0 ; 0];  % initial condition for taudi


%% Controller Parameters
    
SP.Kp=2e3*diag([1 1 1]); % The proportional controller gain Kp
SP.Kd=1e5*diag([1 1 1]); % The  derivative  controller gain Kd
SP.Ktaup=5e1*diag([1 10 0.2]); % The proportian force controller gain 
SP.Ktaui=1e3*diag([1 10 0.2]); % The integral force controller gain Ki

end