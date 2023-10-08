%% 
%   Parallel Robots: Mechanics and Control
%   Copyright Hamid D. Taghirad 2013
%
%       This program plots the simulation result.
%
function [Out] = PlotData(t,X,SP)
%PlotStructure(t,X,SP),
t=t' ; y=X';

%% System variables
for j=1:max(size(t));
clc
disp ([' Steps remained to plot the data:  ', num2str(max(size(t)) -j)]); % Show the process time

[yd(:,j),dyd(:,j)]=TP_cubic_s(t(j),SP);
Y(:,j)    = y(1:6,j) ;
Ydot(:,j) = y(7:12,j) ;
[KC]             = Kinematic_Configuration(Y(:,j),Ydot(:,j),SP) ;
[P_Dynamic_Mats] = Parts_Dynamic_Matrixes(SP,KC) ;
[M_Dynamic_Mats] = Manipulator_Dynamic_Matrixes(SP,P_Dynamic_Mats) ;
[M_Dynamic_Mats] = Control(t(j),y(:,j),SP,KC,M_Dynamic_Mats,P_Dynamic_Mats) ;
PD(:,j)  = M_Dynamic_Mats.PD;
JF(:,j)  = M_Dynamic_Mats.JF;
CF(:,j)  = M_Dynamic_Mats.CF;
end

Out.t=t;
Out.y=y;
Out.yd=yd;
Out.JF=JF;
Out.PD=PD;
Out.CF=CF;
 
%% Trajectory
figure(1)
subplot(211)
plot(t,yd(1,:),t,yd(2,:),'--', t,yd(3,:),'-.',...
    t,y(3,:),'r-.' ),grid on
%    t,y(1,:),'b',t,y(2,:),'dg--', t,y(3,:),'r-.' ),grid on
xlabel('time(sec)')
ylabel('Position m')
title('Tracking Performance')
legend('x_d','y_d','z_d')
subplot(212)
plot(t,yd(4,:)*180/pi,t,yd(5,:)*180/pi,'--', t,yd(6,:)*180/pi,'-.',...
      t,y(6,:)*180/pi,'r-.'),grid on
%      t,y(4,:)*180/pi,'b',t,y(5,:)*180/pi,'dg--', t,y(6,:)*180/pi,'r-.'),grid on
%axis([0 2 -0.5 1])
xlabel('time(sec)')
ylabel('Orientation deg.')
legend('\theta_x_d','\theta_y_d','\theta_z_d')

%% Tracking Errors
figure(2)
subplot(211)
plot(t,yd(1,:)-y(1,:), ...
    t,yd(2,:)-y(2,:),'--', ...
    t,yd(3,:)-y(3,:),'-.'),grid on
xlabel('time(sec)')
ylabel('Position Error m')
title('Tracking Performance')
legend('e_x','e_y','e_z')
subplot(212)
plot(t,(yd(4,:)-y(4,:))*180/pi, ...
    t,(yd(5,:)-y(5,:))*180/pi,'--', ...
    t,(yd(6,:)-y(6,:))*180/pi,'-.' ),grid on
xlabel('time(sec)')
ylabel('Orientation Error deg.')
legend('e_\theta_x','e_\theta_y','e_\theta_z')

%% Cartesian forces
figure(3)
subplot(211)
plot(t,CF(1,:)/1000,t,CF(2,:)/1000,'--',t,CF(3,:)/1000,'-.'),grid on
xlabel('time(sec)')
ylabel('KN')
title('Cartesian Forces')
legend('F_x','F_y','F_z')

subplot(212)
plot(t,CF(4,:)/1000,t,CF(5,:)/1000,'--',t,CF(6,:)/1000,'-.'),grid on
xlabel('time(sec)')
ylabel('KN.m')
title('Cartesian Torques')
legend('\tau_x','\tau_y','\tau_z')

%% Joint forces
figure(4)
subplot(211)
plot(t,JF(1,:)/1000,t,JF(2,:)/1000,'--',t,JF(3,:)/1000,'-.'), grid on
title('Actuator Force');
xlabel('time(sec)')
ylabel('Forces (KN)')
legend('F_1','F_2','F_3')

subplot(212)
plot(t,JF(4,:)/1000,t,JF(5,:)/1000,'--',t,JF(6,:)/1000,'-.'), grid on
title('Actuator Force');
xlabel('time(sec)')
ylabel('Forces (KN)')
legend('F_4','F_5','F_6')

end

