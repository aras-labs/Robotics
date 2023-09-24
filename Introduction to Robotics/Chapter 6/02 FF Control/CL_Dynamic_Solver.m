%% 
% 	An Introduction to Robotics, 
%   By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
%   Copyright ARAS @2023
%
%   This code Runs the closed loop simulation of 3R manipulator
%   with PD controller with Feed Forward Term
%   
%% Initializing
clear ;
clc ; clf ; cla;
tic

%% Determining the Structure 

[SP] = Structural_Parameters() ;

%% Solve The Dynamic Equation

[t,q] = ode23s(@(t,qs) CL_Dynamics(t,qs,SP), SP.tspan, ...
    [SP.q0 ; SP.qd0; SP.qi0; SP.xi0] ... % initial condition for intergrations
    ,odeset('OutputFcn','odeplot','OutputSel',[1:3])) ;
toc
%% Ploting and Exporting the Results
[Out]=PlotData(t,q,SP);

% Figure 1 --> Figure 6.26 of the book
% Figure 2 --> Figure 6.28 of the book
% Figure 3 --> Figure 6.27 of the book