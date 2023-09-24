%% 
% 	An Introduction to Robotics, 
%   By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
%   Copyright ARAS @2023
%
%   This function generates all the requires parameters of the 
%   programs in a structure format.
%
function [SP] = Structural_Parameters()
%% 
deg2rad = pi/180 ;
rad2deg = 180/pi;

%% Kinematics and Dynamics Parameters

SP.n   = 3;                 % No. of degrees of Freedom
SP.a   = [1 ; 0.7; 0.3];    %   Link lengths
SP.m = [10; 7; 3];          % the Mass of the E.F. (Kg)
SP.g = -9.81;               % Gravity Acceleration (m/s^2)
SP.pert = 0.50;             % 50% parameter perterbation

%% Desired Trajectory 

%   The desired trajectory initial points
%     Time  \theta_1    \theta_2    \theta
SP.xd=[
        0    00*deg2rad    00*deg2rad   00*deg2rad
        2    90*deg2rad   -60*deg2rad   30*deg2rad
        4    0*deg2rad      0*deg2rad   0*deg2rad
        6    0*deg2rad      0*deg2rad   0*deg2rad
        8    0*deg2rad      0*deg2rad   0*deg2rad
        10   0*deg2rad      0*deg2rad   0*deg2rad
        ];
    
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
    1.001   50       0       0
    2       50       0       0
    2.001   50      20       0
    3       50      20       0
    3.001   50      20       2
    10      50      20       2
  ];
SP.time=taud(:,1);   
SP.taud=taud(:,2:4);    
SP.Isd=1;       % 0:no disturbance;   1:Is disturbance
%%
%  Actuator limitations
%
SP.tau_limit=[150; 60; 5];    % Torque limits
SP.Istau=0;      % 0:no saturation;   1:With Saturation

%% Simulation Parameters
SP.ts = 0 ; SP.tf = 4 ;
SP.tspan=[SP.ts SP.tf];
SP.q0 =  [0 ; 0 ; 0]*deg2rad ;
SP.qd0 = [0 ; 0 ; 0]*deg2rad;
SP.qi0 = [0 ; 0 ; 0]*deg2rad ;  % initial condition for qi
SP.xi0 = [0 ; 0 ; 0]*deg2rad ;  % initial condition for qdi

%% Controller Parameters
    
SP.Kp=1e4*diag([8 3 0.2]); % The proportional controller gain Kp
SP.Kd=1e4*diag([8 3 0.2]); % The  derivative  controller gain Kd
SP.Ki=1e5*diag([8 3 0.2]); % The  integral controller gain Ki

end