%% 
% 	An Introduction to Robotics, 
%   By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
%   Copyright ARAS @2023
%
%   This function generates all the required parameters of
%   the programs in a structured format. 
%   
%
function [SP] = Structural_Parameters()
%% 
deg2rad = pi/180 ;
rad2deg = 180/pi;

%% Kinematics and Dynamics Parameters

SP.n   = 3;                 % No. of degrees of Freedom
SP.a   = [1 ; 0.7; 0.3];    % Link lengths (m)
SP.m = [10; 7; 3];          % the Mass of the E.F. (Kg)
SP.g = -9.81 ;              % Gravity Acceleration (m/s^2)

%% Desired Trajectory 

%   The desired trajectory initial points
%     Time  \theta_1    \theta_2    \theta
SP.xd=[
        0    00*deg2rad  00*deg2rad   00*deg2rad
        2    60*deg2rad  -30*deg2rad   30*deg2rad
        4    0*deg2rad  0*deg2rad   0*deg2rad
        6    0*deg2rad  0*deg2rad   0*deg2rad
        8    0*deg2rad  0*deg2rad   0*deg2rad
        10   0*deg2rad  0*deg2rad   0*deg2rad
        ];
    
%% Noise Amplitude
SP.noise_q  = 1e-12*[60;30;30]*deg2rad*0;  % noise
SP.noise_dq = 1e-6*[0.2;0.2;0.02]*0;

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
SP.taud=taud(:,2:4);    % no disturbance
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
SP.ts = 0 ; SP.tf = 4 ;
SP.tspan=[SP.ts SP.tf];
SP.q0 = [0 ; 0 ; 0]*deg2rad ;
SP.qd0 = [0 ; 0 ; 0]*deg2rad;

%% Controller Parameters
    
SP.Kp=1e4*diag([8 3 0.2]); % The proportional controller gain Kp
SP.Kd=1e4*diag([8 3 0.2]); % The  derivative  controller gain Kd
%SP.Kp=1e3*diag([8 3 0.2]); % The proportional controller gain Kp with noise
%SP.Kd=1e2*diag([8 3 0.2]); % The  derivative  controller gain Kd
end