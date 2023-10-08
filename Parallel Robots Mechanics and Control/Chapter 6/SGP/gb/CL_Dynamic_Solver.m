%% 
%   Parallel Robots: Mechanics and Control
%   Copyright Hamid D. Taghirad 2013
%
%   This program solves the inverse dynamics of 
%	the Stewart-Gough platform using explicit formulation 
%
%function [] = CL_Dynamic_Solver()


%% Initializing

clear ;
clc ; clf
%close all ;

%% Determining the Structure 

[Struct_Param] = Structural_Parameters() ;

%% Determine Time Span

ts = 0 ;
tf = 2 ;
tspan=[ts tf];

X0 = [0 ; 0 ; +1 ; 0 ; 0 ; 0] ;
Xdot0 = [0 ; 0 ; 0 ; 0 ; 0 ; 0] ;

%% Solve The Dynamic Equation

[t,X] = ode23(@(t,X_s) CL_Dynamic_Equation(t,X_s,Struct_Param),...
    tspan,[X0 ; Xdot0],odeset('OutputFcn','odeplot','OutputSel',[3])) ;

%% Ploting the Results

[Out]=PlotData(t,X,Struct_Param);