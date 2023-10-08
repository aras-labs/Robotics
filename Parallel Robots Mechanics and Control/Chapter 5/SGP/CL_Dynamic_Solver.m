%% 
%   Parallel Robots: Mechanics and Control
%   Copyright Hamid D. Taghirad 2013
%
%       This program solves the inverse dynamics of 
%	the Stewart-Gough platform using explicit formulation 
%	in a closed-loop structure.
%

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

%PlotStructure(t,X,Struct_Param) ;
t=t' ; y=X';
%   Calculate the other system variables
for j=1:max(size(t));
[yd(:,j),dyd(:,j)]=TP_cubic_s(t(j),Struct_Param);
end


figure (1)
subplot(211)
plot(t,yd(1,:),t,yd(2,:),'--', t,yd(3,:),'-.' ),grid on
xlabel('time(sec)')
ylabel('position m')
title('Tracking Performance')
legend('x_d','y_d','z_d')
subplot(212)
plot(t,yd(4,:)*180/pi,t,yd(5,:)*180/pi,'--', t,yd(6,:)*180/pi,'-.'),grid on
%axis([0 2 -0.5 1])
xlabel('time(sec)')
ylabel('orientation deg.')
legend('\theta_x_d','\theta_y_d','\theta_z_d')
figure (2)
subplot(211)
plot(t,yd(1,:)-y(1,:), ...
    t,yd(2,:)-y(2,:),'--', ...
    t,yd(3,:)-y(3,:),'-.'),grid on
xlabel('time(sec)')
ylabel('position error m')
title('Tracking Performance')
legend('e_x','e_y','e_z')
subplot(212)
plot(t,(yd(4,:)-y(4,:))*180/pi, ...
    t,(yd(5,:)-y(5,:))*180/pi,'--', ...
    t,(yd(6,:)-y(6,:))*180/pi,'-.' ),grid on
xlabel('time(sec)')
ylabel('orientation error deg.')
legend('e_\theta_x','e_\theta_y','e_\theta_z')
% figure(3)
% plot(t,F), grid on
% title('Actuator Forces');
% xlabel('time(sec)')
% ylabel('Forces (N)')
%legend('F_1','F_2','F_3','F_4','F_5','F_6')
% 
% figure(4)
% subplot(111)
% plot(t,CF), grid on
% title('Cartesian Forces');
% xlabel('time(sec)')
% ylabel('Forces')
% 
% figure;
% subplot(211)
% plot(t,y(:,3),t,y(:,2),'--',t,y(:,1),'-.'),grid on
% xlabel('time(sec)')
% ylabel('positions m')
% legend('z','y', 'x','location','Best')
% 
% subplot(212)
% plot(t,y(:,4)*180/pi,t,y(:,5)*180/pi,'--',t,y(:,6)*180/pi,'-.'),grid on
% xlabel('time(sec)')
% ylabel('Orientation degrees')
% axis([ts tf -0.5 1])
% legend('\theta_x','\theta_y', '\theta_z','location','Best')
%end