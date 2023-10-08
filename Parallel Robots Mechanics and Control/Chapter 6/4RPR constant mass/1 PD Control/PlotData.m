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
disp ([' Steps remained to plot the data:  ', num2str(max(size(t))-j) ]); % Show the process time

[yd(:,j),dyd(:,j)]=TP_cubic_s(t(j),SP);
Y(:,j)    = y(1:3,j) ;
Ydot(:,j) = y(4:6,j) ;
[KC]             = Kinematic_Configuration(Y(:,j),Ydot(:,j),SP) ;
[P_Dynamic_Mats] = Parts_Dynamic_Matrixes(SP,KC) ;
[M_Dynamic_Mats] = Manipulator_Dynamic_Matrixes(SP,P_Dynamic_Mats) ;
[M_Dynamic_Mats] = PD_Control(t(j),y(:,j),SP,KC,M_Dynamic_Mats,P_Dynamic_Mats) ;
[SP,M_Dynamic_Mats] = Redundancy_Resolution(SP,KC,M_Dynamic_Mats) ;
JF(:,j) = M_Dynamic_Mats.JF;
CF(:,j) = M_Dynamic_Mats.CF;
JFP(:,j)= M_Dynamic_Mats.JFP;
end
Out.t=t;
Out.y=y;
Out.yd=yd;
Out.JF=JF;
Out.JFP=JFP;
Out.CF=CF;

figure(1)
subplot(311)
plot(t,yd(1,:),t,y(1,:),':','linewidth',1),grid on
xlabel('time(sec)')
ylabel('x_G (m)')
title('Tracking Performance')
legend('Desired Trajectory', 'Output Trajectory')
subplot(312)
plot(t,yd(2,:),t,y(2,:),':','linewidth',1),grid on
xlabel('time(sec)')
ylabel('y_G (m)')
subplot(313)
plot(t,yd(3,:)*180/pi,t,y(3,:)*180/pi,':','linewidth',1),grid on
xlabel('time(sec)')
ylabel('\phi (degrees)')


figure(2)
subplot(311)
plot(t,yd(1,:)-y(1,:)),grid on
title('Tracking Errors')
xlabel('time(sec)')
ylabel('e_x (m)')
%title([' The perturbation in the model parameters is %',num2str((1-Par.pert)*100)]);
%legend('Macro', 'Location', 'SouthWest') 
subplot(312)
plot(t,yd(2,:)-y(2,:)),grid on
xlabel('time(sec)')
ylabel('e_y (m)')
subplot(313)
plot(t,(yd(3,:)-y(3,:))*180/pi),grid on
xlabel('time(sec)')
ylabel('e_\phi  (degrees)')

figure(3)
subplot(221)
plot(t,JF(1,:),'-.', t, JFP(1,:)), grid on
title('Actuator Force 1');
xlabel('time(sec)')
ylabel('Newtons')
legend('Basic Solution', 'Positive Tension', 'Location', 'Best')
subplot(222)
plot(t,JF(2,:),'-.', t, JFP(2,:)), grid on
title('Actuator Force 2');
xlabel('time(sec)')
ylabel('Newtons')
subplot(223)
plot(t,JF(3,:),'-.', t, JFP(3,:)), grid on
title('Actuator Force 3');
xlabel('time(sec)')
ylabel('Newtons')
subplot(224)
plot(t,JF(4,:),'-.', t, JFP(4,:)), grid on
title('Actuator Force 4');
xlabel('time(sec)')
ylabel('Newtons')

figure(4)
subplot(311)
plot(t,CF(1,:)), grid on
xlabel('time(sec)')
ylabel('Newtons')
title('Cartesian Forces F_x');

subplot(312)
plot(t,CF(2,:)), grid on
xlabel('time(sec)')
ylabel('Newtons')
%legend('Fdy', 'IDC_y', 'PID_y', 'F_y')
title('Cartesian Forces F_y');
subplot(313)
plot(t,CF(3,:)), grid on
xlabel('time(sec)')
ylabel('N.m')
title('Cartesian torque \tau_\phi');
%legend('Fd', 'PD','Location', 'NorthWest')

return
