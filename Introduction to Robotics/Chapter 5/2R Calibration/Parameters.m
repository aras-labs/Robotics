%% 
% 	An Introduction to Robotics, 
%   By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
%   Copyright ARAS @2023
%
%   This program sets the Physical Parameters of the 2R Robot
%   in a structure format 
%
deg2rad=pi/180; rad2deg=180/pi;
Par.t0=0; Par.tf=3;          % Simulation time
Par.tspan=[Par.t0 Par.tf];   % Time span
% The geometric and inertial parameters of the robot 
Par.a=[1;0.7];           % The length of the links
Par.pert=1.0;                % Pertutbation for parameters
Par.m=[10;7];              % The Mass of the links m
Par.mc=[1;0.7]/2;        % The center of mass of links
Par.I=1/12*Par.m.*Par.a.^2;  % The Moment of interia of the links 
                             %  (Slender Bars)
Par.q0=[0;0;0]*deg2rad;      % The initial condition for q
Par.qf=[-60;-30;0]*deg2rad;   % The final condition for q
Par.dq0=[0;0]*deg2rad;     % The initial condition for \dot q
Par.ddq0=[0;0]*deg2rad;    % The initial condition for \ddot q
Par.g=9.8108;                % Gravity acceleration
%       Actuator Dynamics
Par.eta=[200; 200];       % Gearbox ratios
Par.b=1e-1*[1;1];           % Joint viscous friction
Par.bm=1e-3*[1;1];         % Joint viscous friction
Par.Im=1e-2*[10;7];         % Joint viscous friction

t0=Par.t0; timef=Par.tf;        % Simulation time
tspan=Par.tspan;             % Time span
Par.KP=1e6*diag([1,1]);   % The proportional controller gain KP
Par.KV=1e8*diag([1,1]);   % The derivative controller gain KV
%Par.KI=0*eye(3);            % The integral controller gain KI
%%
%   The desired trajectory initial points
%       Time    q_1             q_2             q_3 
Par.qd=[t0      0               0               0 
        1     -30*deg2rad      -90*deg2rad    0
        2     -90*deg2rad      -30*deg2rad     0
        timef   0               0               0
        40      0               0               0
        50      0               0               0
        100     0               0                0];

a=Par.a; pert=Par.pert; g=9.81;
m=pert*Par.m; mc=pert*Par.mc; I=pert*Par.I; 

