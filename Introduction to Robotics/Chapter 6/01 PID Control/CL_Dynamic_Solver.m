%% 
% 	An Introduction to Robotics, 
%   By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
%   Copyright ARAS @2023
%
%   This code Runs the closed loop simulation of 3R manipulator
%   with PID controller
%   
%% Initializing
clear ;
clc ; clf ; cla;

%% Determining the Structure 

[SP] = Structural_Parameters() ;

%% Solve The Dynamic Equation

[t,q] = ode23s(@(t,qs) CL_Dynamics(t,qs,SP), SP.tspan, ...
    [SP.q0 ; SP.qd0; SP.qi0; SP.xi0] ... % initial condition for intergrations
    ,odeset('OutputFcn','odeplot','OutputSel',[1:3])) ;

%% Ploting and Exporting the Results
[Out]=PlotData(t,q,SP);

% Figure 1 --> Figure 6.19 of the book
% Figure 2 --> Figure 6.20 of the book
% Figure 3 --> Figure 6.21 of the book
