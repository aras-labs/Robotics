%% 
%   Parallel Robots: Mechanics and Control
%   Copyright Hamid D. Taghirad 2013
%
%       This program solves the inverse dynamics of 
%	the Planar Cable Manipulator using explicit formulation 
%	in a closed-loop structure.
%
%% Initializing
clear ;
clc ;
clf ;

%% Determining the Structure 

[SP] = Structural_Parameters() ;

%% Solve The Dynamic Equation

[t,X] = ode45(@(t,X_s) CL_Dynamic_Equation(t,X_s,SP),SP.tspan,[SP.X0 ; SP.Xdot0],odeset('OutputFcn','odeplot')) ;

%% Ploting and Exporting the Results
[Out]=PlotData(t,X,SP);
