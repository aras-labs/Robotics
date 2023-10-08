%% 
%   Parallel Robots: Mechanics and Control
%   Copyright Hamid D. Taghirad 2013
%
%   This program solves the inverse dynamics of 
%	the Stewart-Gough platform using explicit formulation 
%
function [Out] = CL_Dynamic_Solver()


%% Initializing

clear all;
clc ; clf;

%% Determining the Structure 

[SP] = Structural_Parameters() ;

%% Solve The Dynamic Equation

[t,X] = ode23(@(t,X_s) CL_Dynamic_Equation(t,X_s,SP),SP.tspan, ...
[SP.X0 ; SP.Xdot0],odeset('OutputFcn','odeplot','OutputSel',[3])) ;

%% Ploting and Exporting the Results

[Out]=PlotData(t,X,SP);

end