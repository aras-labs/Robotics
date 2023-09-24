%% 
% 	An Introduction to Robotics, 
%   By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
%   Copyright ARAS @2023
%
%   This function plots the simulation result.
%
function [Out] = PlotData(t,X,SP)

disp ('Recalculating the simulated data ...'); % Show the process time
tic
%
deg2rad = pi/180 ;
rad2deg = 180/pi;
t=t' ; y=X';
%   Calculate the other system variables
for j=1:max(size(t));
[yd(:,j),dyd(:,j)]=TP_quintic(t(j),SP);
Y(:,j)    = y(1:3,j) ;
Ydot(:,j) = y(4:6,j) ;
Yi(:,j)   = y(7:9,j) ;
qi(:,j)   = y(10:12,j) ;
[Dynamic_Mats] = Dynamic_Matrices(y(:,j), SP) ;
[Tau] = PID_Control(t(j),y(:,j),SP) ;
PID(:,j) = Tau.PID;
taud(:,j) = Tau.taud;
end
Out.t=t;
Out.y=y; Out.yd=yd;
Out.tau=PID; Out.taud=taud;
toc

figure(1)
subplot(311)
plot(t,yd(1,:)*rad2deg,t,y(1,:)*rad2deg,':k'),grid on
xlabel('time (sec)')
ylabel('q_1 (deg)')
title('Tracking Performance')
legend('q_d', 'q_i')
subplot(312)
plot(t,yd(2,:)*rad2deg,t,y(2,:)*rad2deg,':k','linewidth',1),grid on
xlabel('time (sec)')
ylabel('q_2 (deg)')
subplot(313)
plot(t,yd(3,:)*rad2deg,t,y(3,:)*rad2deg,':k','linewidth',1),grid on
xlabel('time(sec)')
ylabel('q_3 (deg)')
set(findall(figure(1),'type','line'),'linewidth',1.5)


figure(2)
subplot(311)
plot(t,(yd(1,:)-y(1,:))*180/pi,'k'),grid on
title('Tracking Errors')
xlabel('time (sec)')
ylabel('e_1 (deg)')
%title([' The perturbation in the model parameters is %',num2str((1-Par.pert)*100)]);
subplot(312)
plot(t,(yd(2,:)-y(2,:))*180/pi,'k'),grid on
xlabel('time (sec)')
ylabel(['e_2 (deg)'])
subplot(313)
plot(t,(yd(3,:)-y(3,:))*180/pi,'k'),grid on
xlabel('time (sec)')
ylabel('e_3 (deg)')
  set(findall(figure(2),'type','line'),'linewidth',1.5)

figure(3)
subplot(311)
plot(t,PID(1,:),':k', t, taud(1,:),'-.k'), grid on
title('Actuator Torques');
xlabel('time (sec)')
ylabel('\tau_1 (N.m)')
legend('\tau_{pid}', '\tau_d')
subplot(312)
plot(t,PID(2,:),':k', t, taud(2,:),'-.k'), grid on
xlabel('time (sec)')
ylabel('\tau_2 (N.m)')
subplot(313)
plot(t,PID(3,:),':k', t, taud(3,:),'-.k'), grid on
xlabel('time (sec)')
ylabel('\tau_3 (N.m)')
  set(findall(figure(3),'type','line'),'linewidth',1.5)

 return
