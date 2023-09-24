%% 
% 	An Introduction to Robotics, 
%   By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
%   Copyright ARAS @2023
%
%   This program illustrated transient response of a second order system
%   for different values of \zeta and \omega_n
%
%% Parameter Definitions
clear; clc; clf
s=tf('s');
zeta=[0; 0.2155 ;0.4; 0.707; 1; 2]; omega=1;
sys1=1/(s^2+2*zeta(1)*omega*s+omega^2);
sys2=1/(s^2+2*zeta(2)*omega*s+omega^2);
sys3=1/(s^2+2*zeta(3)*omega*s+omega^2);
sys4=1/(s^2+2*zeta(4)*omega*s+omega^2);
sys5=1/(s^2+2*zeta(5)*omega*s+omega^2);
sys6=1/(s^2+2*zeta(6)*omega*s+omega^2);

t=0.:0.01:8*pi;
y1=step(sys1, t); y2=step(sys2, t);
y3=step(sys3, t); y4=step(sys4, t);
y5=step(sys5, t); y6=step(sys6, t);
t=t/2/pi;

%%   Final Plot
%    Figure 6.6 of the book
plot(t,y1, t, y2, '-.', ...
    t,y3, '--m', t, y4, '-k', ...
    t,y5, '-.', t, y6, '--b' ), grid
set(findall(figure(1),'type','line'),'linewidth',2)
legend('\xi=0','\xi=0.2155','\xi=0.4','\xi=0.707','\xi=1',...
    '\xi=2')
xlabel('\omega_n t (sec)')
ylabel('Output')
