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

[yd(:,j),dyd(:,j)]=TP_cubic_s(t(j),SP);
Y(:,j)    = y(1:3,j) ;
Ydot(:,j) = y(4:6,j) ;
[KC]             = Kinematic_Configuration(Y(:,j),Ydot(:,j),SP) ;
[P_Dynamic_Mats] = Parts_Dynamic_Matrixes(SP,KC) ;
[M_Dynamic_Mats] = Manipulator_Dynamic_Matrixes(SP,P_Dynamic_Mats) ;
[M_Dynamic_Mats] = SC_Control(t(j),y(:,j),SP,KC,M_Dynamic_Mats,P_Dynamic_Mats) ;
M_Dynamic_Mats.JF=(pinv(KC.J')*M_Dynamic_Mats.CF)';      % base solution tau_0
[SP,M_Dynamic_Mats] = Redundancy_Resolution(SP,KC,M_Dynamic_Mats) ;
PD(:,j) = M_Dynamic_Mats.PD;
JF(:,j) = M_Dynamic_Mats.JF;
CF(:,j) = M_Dynamic_Mats.CF;
JFP(:,j)= M_Dynamic_Mats.JFP;
Fe(:,j) = M_Dynamic_Mats.Fe;
end
Out.t=t;
Out.y=y;
Out.yd=yd;
Out.JF=JF;
Out.JFP=JFP;
Out.PD=PD;
Out.CF=CF;
Out.Fe=Fe;

figure(1)
subplot(311)
plot(t,yd(1,:),t,y(1,:),'-.'),grid on
xlabel('time(sec)')
ylabel('x_G (m)')
title('Tracking Performance')
% title(['K_P= ',num2str(KP(1,1),'%2.1g'),...
%      ', K_V= ',num2str(KV(1,1),'%2.1g'),...
%      ', K_p= ',num2str(Kp(1,1),'%2.1g'),...
%      ', K_v= ',num2str(Kv(1,1),'%2.1g'),')'])
% %     ', K_I= ',num2str(KI(1,1),'%2.1g'),')'])
legend('Desired Trajectory', 'Output Trajectory','Location','Northwest')
subplot(312)
plot(t,yd(2,:),t,y(2,:),'-.'),grid on
xlabel('time(sec)')
ylabel('y_G (m)')
subplot(313)
plot(t,yd(3,:)*180/pi,t,y(3,:)*180/pi,'-.'),grid on
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
plot(t,JF(1,:)/1000,'-.', t, JFP(1,:)/1000), grid on
title('Actuator Force 1');
xlabel('time(sec)')
ylabel('KN')
legend('Basic Solution', 'Positive Tension', 'Location', 'Best')
subplot(222)
plot(t,JF(2,:)/1000,'-.', t, JFP(2,:)/1000), grid on
title('Actuator Force 2');
xlabel('time(sec)')
ylabel('KN')
subplot(223)
plot(t,JF(3,:)/1000,'-.', t, JFP(3,:)/1000), grid on
title('Actuator Force 3');
xlabel('time(sec)')
ylabel('KN')
subplot(224)
plot(t,JF(4,:)/1000,'-.', t, JFP(4,:)/1000), grid on
title('Actuator Force 4');
xlabel('time(sec)')
ylabel('KN')

figure(4)
subplot(311)
plot(t,CF(1,:)/1000), grid on
xlabel('time(sec)')
ylabel('KN')
title('Cartesian Forces F_x');
subplot(312)
plot(t,CF(2,:)/1000), grid on
xlabel('time(sec)')
ylabel('KN')
title('Cartesian Forces F_y');
subplot(313)
plot(t,CF(3,:)/1000), grid on
xlabel('time(sec)')
ylabel('KN.m')
title('Cartesian Torque \tau_\phi');
%legend('Fd', 'PD','Location', 'NorthWest')

figure(5)
subplot(211)
plot(t,(yd(1,:)-y(1,:)),t,Fe(1,:)/1000,'--'), grid on
title('Force vs Position Error');
xlabel('Time (sec)')
ylabel('Error (m), Force (KN)')
legend('Tracking Error', 'Force','Location','Northwest')
subplot(212)
plot((yd(1,:)-y(1,:)),Fe(1,:)/1000,'.'), grid on
xlabel('Position Error (m)')
ylabel('Force (KN)')

return
