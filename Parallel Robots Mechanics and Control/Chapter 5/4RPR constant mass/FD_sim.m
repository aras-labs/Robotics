%% 
%   Parallel Robots: Mechanics and Control
%   Copyright Hamid D. Taghirad 2013
%
%   This program solves the forward dynamics of 
%   the planar cable manipulator using implicit 
%   Newton-Euler formulation for a typical trajectory.
%
%%
clear all;

global Par      % Use all parameters in a Global Structure
%   initialize the simulation parameters
deg2rad=pi/180;
rad2deg=180/pi;
t0=0; tf=350;
tspan=[t0 tf];
% The orientation of Ai, Bi, ai, and bi's in its at central position
Par.Ath=[-135;-45;45;135]*deg2rad; 
Par.Bth=[-45;-135;135;45]*deg2rad;
Par.pert=1.0;                % {ertutbation for parameters
Par.RA=900;                  % the Ai's circle radius 
Par.RB=10;                   % the Bi's circle radius 
Par.M=2500;                  % The Mass of moving platform M
Par.I=3.5e5;                 % The moment of inertia of the MP
Par.rho=0.215;               % The limb's density (Kg/m
Par.F_old=zeros(4,1);        % Initial guess for redundancy resolution
Par.x0=zeros(6,1);           % The initial condition for x
Par.xp0=zeros(6,1);          % The initial condition for xdot
Par.time=0;
Par.index=0;
Par.F=[1e3;1e3;1e3;1e3];
Par.Fd=[0;0;0];
Par.g=[0;0;-9.81];
x0=Par.x0;
x0(1)=1;x0(2)=1;
xp0=Par.xp0;

%%
%   Find consistent initial conditions for the implicit ode
[x0,xp0] = decic(@ID_4RPR,t0,x0,[]',xp0,[]');

% Solves the implicit ode
[t,x] = ode15i(@ID_4RPR,tspan,x0,xp0);

%%
% plots the result
%
subplot(311)
plot(t,x(:,1)),grid on
xlabel('time(sec)')
ylabel('x_G m')
axis([0 tf -1 1])

subplot(312)
plot(t,x(:,2)),grid on
xlabel('time(sec)')
ylabel('y_G m')

subplot(313)
plot(t,x(:,3)*180/pi),grid on
xlabel('time(sec)')
ylabel('\phi degrees')
axis([0 tf -1 1])

