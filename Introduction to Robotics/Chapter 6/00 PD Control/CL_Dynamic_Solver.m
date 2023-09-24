%% 
% 	An Introduction to Robotics, 
%   By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
%   Copyright ARAS @2023
%
%   This code Runs the closed loop simulation of a 3R manipulator
%   with a PD controller.


%   
%% Parameter Definitions
clear ;
clc ; clf ; cla;

%% Determining the Structure 

[SP] = Structural_Parameters() ;

%% Solve The Dynamic Equation

[t,q] = ode23s(@(t,qs) CL_Dynamics(t,qs,SP), SP.tspan, ...
    [SP.q0 ; SP.qd0],odeset('OutputFcn','odeplot','OutputSel',[1:3])) ;

%% Ploting and Exporting the Results
[Out]=PlotData(t,q,SP);

% Figure 2 --> Figure 6.22 of the book
% Figure 3 --> Figure 6.23 of the book