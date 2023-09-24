%% 
% 	An Introduction to Robotics, 
%   By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
%   Copyright ARAS @2023
%
%   This function plots the simulation result.
%
function [Out] = PlotData(t,X,SP)

t=t' ; y=X';
%clc
disp ('Please be patient to calculate the simulated data ...'); % Show the process time
tic

%   Calculate the other system variables
for j=1:max(size(t));
[yd(:,j),dyd(:,j)]=TP_quintic(t(j),SP);
Y(:,j)    = y(1:3,j) ;
Ydot(:,j) = y(4:6,j) ;
%Yi(:,j)   = y(7:9,j) ;
%qi(:,j)   = y(10:12,j) ;
[Dynamic_Mats] = Dynamic_Matrices(y(:,j), SP) ;
[Tau] = PD_Control(t(j),y(:,j),SP) ;
PD(:,j) = Tau.PD;
taud(:,j) = Tau.taud;
end
Out.t=t;
Out.y=y; Out.yd=yd;
Out.tau=PD; Out.taud=taud;
toc

figure(1)
subplot(311)
plot(t,yd(1,:)*180/pi,t,y(1,:)*180/pi,'-.k','linewidth',1),grid on
xlabel('time (sec)')
ylabel('\theta_1 (deg)')
title('Tracking Performance')
legend('Desired Trajectory', 'Output Trajectory')
subplot(312)
plot(t,yd(2,:)*180/pi,t,y(2,:)*180/pi,'-.k','linewidth',1),grid on
xlabel('time (sec)')
ylabel('\theta_2 (deg)')
subplot(313)
plot(t,yd(3,:)*180/pi,t,y(3,:)*180/pi,'-.k','linewidth',1),grid on
xlabel('time(sec)')
ylabel('\theta_3 (deg)')
  set(findall(figure(1),'type','line'),'linewidth',2)

figure(2)
subplot(311)
plot(t,(yd(1,:)-y(1,:))*180/pi,'k'),grid on
title('Tracking Errors')
xlabel('time (sec)')
ylabel('e_1 (deg)')

subplot(312)
plot(t,(yd(2,:)-y(2,:))*180/pi,'k'),grid on
xlabel('time (sec)')
ylabel(['e_2 (deg)'])
subplot(313)
plot(t,(yd(3,:)-y(3,:))*180/pi,'k'),grid on
xlabel('time (sec)')
ylabel('e_3 (deg)')
  set(findall(figure(2),'type','line'),'linewidth',2)

figure(3)
subplot(311)
plot(t,PD(1,:),'k', t, taud(1,:),'-.b'), grid on
title('Actuator Torques');
xlabel('time (sec)')
ylabel('\tau_1 (N.m)')
legend('PD', '\tau_d', 'Location', 'Best')
subplot(312)
plot(t,PD(2,:),'k', t, taud(2,:),'-.b'), grid on
xlabel('time (sec)')
ylabel('\tau_2 (N.m)')
subplot(313)
plot(t,PD(3,:),'k', t, taud(3,:),'-.b'), grid on
xlabel('time (sec)')
ylabel('\tau_3 (N.m)')
  set(findall(figure(3),'type','line'),'linewidth',2)

return
