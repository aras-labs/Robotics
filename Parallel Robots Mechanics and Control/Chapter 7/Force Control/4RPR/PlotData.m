%% 
%   Parallel Robots: Mechanics and Control
%   Copyright Hamid D. Taghirad 2013
%
%       This program plots the simulation result.
%
function [Out] = PlotData(t,X,SP)

t=t' ; y=X';
%   Calculate the other system variables
for j=1:max(size(t));

clc
disp ([' Steps remained to plot the data:  ', num2str(max(size(t)) -j)]); % Show the process time

[Fd(:,j),dFd(:,j)]=TP_cubic_s(t(j),SP);
Y(:,j)    = y(1:3,j) ;
Ydot(:,j) = y(4:6,j) ;
[KC]             = Kinematic_Configuration(Y(:,j),Ydot(:,j),SP) ;
[P_Dynamic_Mats] = Parts_Dynamic_Matrixes(SP,KC) ;
[M_Dynamic_Mats] = Manipulator_Dynamic_Matrixes(SP,P_Dynamic_Mats) ;
[M_Dynamic_Mats] = FIC_Control(t(j),y(:,j),SP,KC,M_Dynamic_Mats,P_Dynamic_Mats) ;
M_Dynamic_Mats.JF=(pinv(KC.J')*M_Dynamic_Mats.CF)';      % base solution tau_0
[SP,M_Dynamic_Mats] = Redundancy_Resolution(SP,KC,M_Dynamic_Mats) ;
PD(:,j) = M_Dynamic_Mats.PD;
JF(:,j) = M_Dynamic_Mats.JF;
CF(:,j) = M_Dynamic_Mats.CF;
JFP(:,j)= M_Dynamic_Mats.JFP;
Fe(:,j) = M_Dynamic_Mats.Fe;
Fm(:,j) = M_Dynamic_Mats.Fm;
FL(:,j) = M_Dynamic_Mats.FL;
Xa(:,j) = M_Dynamic_Mats.Xa;
end
Out.t=t; Out.y=y; %Out.yd=yd;
Out.JF=JF; Out.JFP=JFP;
Out.PD=PD;Out.CF=CF;Out.CF=Fe;
Out.Fm=Fm; Out.FL=FL; Out.Xa=Xa; 



figure(1)
subplot(311)
plot(t,Fd(1,:),t,Fm(1,:),'-.'),grid on
xlabel('time(sec)')
ylabel('F_x (N)')
title('Force Tracking Performance')
legend('Desired Force', 'Output Force', 'Location','Best')
subplot(312)
plot(t,Fd(2,:),t,Fm(2,:),'-.'),grid on
xlabel('time(sec)')
ylabel('F_y (N)')
subplot(313)
plot(t,Fd(3,:),t,Fm(3,:),'-.'),grid on
xlabel('time(sec)')
ylabel('\tau_\phi (N.m)')

figure(2)
subplot(311)
plot(t,Fd(1,:)-Fm(1,:)),grid on
xlabel('time(sec)')
ylabel('e_{Fx} (N)')
title('Force Tracking Error')
%legend('Desired Force', 'Output Force', 'Location','Best')
subplot(312)
plot(t,Fd(2,:)-Fm(2,:)),grid on
xlabel('time(sec)')
ylabel('e_{Fy} (N)')
%title('Force Tracking Error')
subplot(313)
plot(t,Fd(3,:)-Fm(3,:)),grid on
xlabel('time(sec)')
ylabel('e_{\tau} (N.m)')
xlabel('time(sec)')

figure(3)
subplot(311)
plot(t,y(1,:),t,Xa(1,:),'-.'),grid on
xlabel('time(sec)')
ylabel('x Position  (m)')
%title('Secondary Objective')
legend('x', 'x_a', 'Location','Best')
subplot(312)
plot(t,y(2,:),t,Xa(2,:),'-.'),grid on
xlabel('time(sec)')
ylabel('y Position  (m)')
subplot(313)
plot(t,y(1,:),t,Xa(1,:),'-.'),grid on
xlabel('time(sec)')
ylabel('\phi (degrees)')
% figure(2)
% subplot(311)
% plot(t,yd(1,:)-y(1,:)),grid on
% title('Tracking Errors')
% xlabel('time(sec)')
% ylabel('e_x (m)')
% subplot(312)
% plot(t,yd(2,:)-y(2,:)),grid on
% xlabel('time(sec)')
% ylabel('e_y (m)')
% subplot(313)
% plot(t,(yd(3,:)-y(3,:))*180/pi),grid on
% xlabel('time(sec)')
% ylabel('e_\phi  (degrees)')

figure(4)
subplot(221)
plot(t,JF(1,:)/1000,'-.', t, JFP(1,:)/1000), grid on
axis([SP.ts SP.tf -2 2]);
title('Actuator Force 1');
xlabel('time(sec)')
ylabel('KN')
legend('Basic Solution', 'Positive Tension', 'Location', 'Best')
subplot(222)
plot(t,JF(2,:)/1000,'-.', t, JFP(2,:)/1000), grid on
axis([SP.ts SP.tf -2 2]);
title('Actuator Force 2');
xlabel('time(sec)')
ylabel('KN')
subplot(223)
plot(t,JF(3,:)/1000,'-.', t, JFP(3,:)/1000), grid on
axis([SP.ts SP.tf -2 2]);title('Actuator Force 3');
xlabel('time(sec)')
ylabel('KN')
subplot(224)
plot(t,JF(4,:)/1000,'-.', t, JFP(4,:)/1000), grid on
axis([SP.ts SP.tf -2 2]);
title('Actuator Force 4');
xlabel('time(sec)')
ylabel('KN')

figure(5)
subplot(311)
plot(t,CF(1,:)/1000,t,PD(1,:)/1000,'-.'), grid on
axis([SP.ts SP.tf -2 2]);
xlabel('time(sec)')
ylabel('KN')
title('Cartesian Forces F_x');
legend('Total', 'F_{PD}','Location', 'Best')
subplot(312)
plot(t,CF(2,:)/1000,t,PD(2,:)/1000,'-.'), grid on
axis([SP.ts SP.tf -2 2]);
xlabel('time(sec)')
ylabel('KN')
%legend('Fdy', 'IDC_y', 'PID_y', 'F_y')
title('Cartesian Forces F_y');
subplot(313)
plot(t,CF(3,:)/1000,t,PD(3,:)/1000,'-.'), grid on
axis([SP.ts SP.tf -0.2 0.2]);
xlabel('time(sec)')
ylabel('KN.m')
title('Cartesian torque \tau_\phi');
%legend('Fd', 'PD','Location', 'NorthWest')

% figure(5)
% clf
% subplot(211)
% plot(t,Fe(1,:)/1000,t,Fimp(1,:)/1000,'--'), grid on
% axis([SP.ts SP.tf -.1 0.7]);
% xlabel('time(sec)')
% ylabel('Forces in x direction (KN)')
% title('Impedance Dynamics');
% legend('Fe_x', 'Imp. Dyn.', 'Location', 'Northwest')
% % subplot(212)
% % plot((yd(1,:)-y(1,:)),Fe(1,:)/1000,'.'), grid on
% % xlabel('Position Error (m)')
% % ylabel('Force (KN)')

return
