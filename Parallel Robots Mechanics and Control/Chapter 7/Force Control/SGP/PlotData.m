%% 
%   Parallel Robots: Mechanics and Control
%   Copyright Hamid D. Taghirad 2013
%
%       This program plots the simulation result.
%
function [Out] = PlotData(t,X,SP)
t=t' ; y=X';

%% System variables
for j=1:max(size(t));
clc
disp ([' Steps remained to plot the data:  ', num2str(max(size(t)) -j)]); % Show the process time

[Fd(:,j),dFd(:,j)]=TP_cubic_s(t(j),SP);
Y(:,j)    = y(1:6,j) ;
Ydot(:,j) = y(7:12,j) ;
[KC]             = Kinematic_Configuration(Y(:,j),Ydot(:,j),SP) ;
[P_Dynamic_Mats] = Parts_Dynamic_Matrixes(SP,KC) ;
[M_Dynamic_Mats] = Manipulator_Dynamic_Matrixes(SP,P_Dynamic_Mats) ;
[M_Dynamic_Mats] = Control(t(j),y(:,j),SP,KC,M_Dynamic_Mats,P_Dynamic_Mats) ;
PD(:,j) = M_Dynamic_Mats.PD;
JF(:,j) = M_Dynamic_Mats.JF;
CF(:,j) = M_Dynamic_Mats.CF;
Fe(:,j) = M_Dynamic_Mats.Fe;
Fm(:,j) = M_Dynamic_Mats.Fm;
FL(:,j) = M_Dynamic_Mats.FL;
Xa(:,j) = M_Dynamic_Mats.Xa;
end
Out.t=t; Out.y=y;
Out.JF=JF; Out.PD=PD;
Out.CF=CF;Out.CF=Fe;
Out.CF=Fm; Out.FL=FL; Out.Xa=Xa;

 
%% Trajectory
figure(1)
subplot(211)
plot(t,Fd(1,:),'b',t,Fd(2,:),'--', t,Fd(3,:),'r-.',...
    t,Fm(2,:),'--', t,Fm(3,:),'r-.' ),grid on
xlabel('time(sec)')
ylabel('Force (N)')
title('Force Tracking Performance')
legend('Desired Force', 'Output Force', 'Location','Best')
legend('F_x','F_y','F_z','Location','Northwest')
subplot(212)
plot(t,Fd(4,:),t,Fd(5,:),'--', t,Fd(6,:),'-.',...
      t,Fm(4,:),'-',t,Fm(5,:),'--',t,Fm(6,:),'r-.'),grid on
xlabel('time(sec)')
ylabel('Torque (N.m)')
legend('\tau_x','\tau_y','\tau_z','location','Northwest')

%% Tracking Errors
figure(2)
subplot(211)
plot(t,Fd(1,:)-Fm(1,:), ...
    t,Fd(2,:)-Fm(2,:),'--', ...
    t,Fd(3,:)-Fm(3,:),'-.'),grid on
axis([0 1 -10 20])
xlabel('time(sec)')
ylabel('Force Error (N)')
title('Force Tracking Performance')
legend('e_{Fx}','e_{Fy}','e_{Fz}','Location','Northeast')
subplot(212)
plot(t,(Fd(4,:)-Fm(4,:)), ...
    t,(Fd(5,:)-Fm(5,:)),'--', ...
    t,(Fd(6,:)-Fm(6,:)),'-.' ),grid on
xlabel('time(sec)')
ylabel('Torque Error (N.m)')
legend('e_\tau_x','e_\tau_y','e_\tau_z','Location','Northeast')

%% Position Tracking

figure(3)
subplot(311)
plot(t,y(1,:),t,Xa(1,:),'-.'),grid on
xlabel('time(sec)')
ylabel('x Position  (m)')
legend('x', 'x_a', 'Location','Southeast')
subplot(312)
plot(t,y(2,:),t,Xa(2,:),'-.'),grid on
xlabel('time(sec)')
ylabel('y Position  (m)')
legend('y', 'y_a', 'Location','Southeast')
subplot(313)
plot(t,y(3,:)-1,t,Xa(3,:),'-.'),grid on
xlabel('time(sec)')
ylabel('z position (m)')
legend('z-1', 'z_a', 'Location','Southeast')
%% Cartesian forces
figure(4)
subplot(231)
plot(t,CF(1,:)/1000,t,PD(1,:)/1000,'--'), grid on
axis([SP.ts SP.tf -100 400])
xlabel('time(sec)')
ylabel('KN')
title('Cartesian Forces F_x');
legend('Total', 'PD', 'Location', 'Northeast')

subplot(232)
plot(t,CF(2,:)/1000,t,PD(2,:)/1000,'--'), grid on
axis([SP.ts SP.tf -100 400])
xlabel('time(sec)')
ylabel('KN')
title('Cartesian Forces F_y');

subplot(233)
plot(t,CF(3,:)/1000,t,PD(3,:)/1000,'--'), grid on
axis([SP.ts SP.tf -100 400])
xlabel('time(sec)')
ylabel('KN')
title('Cartesian Forces F_z');

subplot(234)
plot(t,CF(4,:)/1000,t,PD(4,:)/1000,'--'), grid on
%axis([SP.ts SP.tf -1 1]);
xlabel('time(sec)')
ylabel('KN.m')
title('Cartesian Torque \tau_x');

subplot(235)
plot(t,CF(5,:)/1000,t,PD(5,:)/1000,'--'), grid on
%axis([SP.ts SP.tf -1 1]);
xlabel('time(sec)')
ylabel('KN.m')
title('Cartesian Torque \tau_y');

subplot(236)
plot(t,CF(6,:)/1000,t,PD(6,:)/1000,'--'), grid on
%axis([SP.ts SP.tf -.1 .2]);
xlabel('time(sec)')
ylabel('KN.m')
title('Cartesian Torque \tau_z');


%% Joint forces
figure(5)
subplot(211)
plot(t,JF(1,:)/1000,t,JF(2,:)/1000,'--',t,JF(3,:)/1000,'-.'), grid on
axis([SP.ts SP.tf -200 200]);
title('Actuator Force');
xlabel('time(sec)')
ylabel('Forces (KN)')
legend('F_1','F_2','F_3')

subplot(212)
plot(t,JF(4,:)/1000,t,JF(5,:)/1000,'--',t,JF(6,:)/1000,'-.'), grid on
axis([SP.ts SP.tf -200 400]);
title('Actuator Force');
xlabel('time(sec)')
ylabel('Forces (KN)')
legend('F_4','F_5','F_6')

end

