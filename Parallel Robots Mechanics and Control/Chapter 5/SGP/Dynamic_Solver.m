%% 
%   Parallel Robots: Mechanics and Control
%   Copyright Hamid D. Taghirad 2013
%
%   This program solves the forward dynamics of 
%   the Stewart-Gough platform using explicit formulation 
%   for a typical initial conditions.
%
%%
function [] = Dynamic_Solver()


%% Initializing

clear ;
clc ;

%% Determining the Parameters Structure 

[Struct_Param] = Structural_Parameters() ;

%% Determine Time Span

ts = 0 ;
tf = 0.5 ;
tspan=[ts tf];

X0 = [0 ; 0 ; +1 ; 0 ; 0 ; 0] ;
Xdot0 = [0 ; 0 ; 0 ; 0 ; 0 ; 0] ;

%% Solve The Dynamic Equation

[t,X] = ode23(@(t,X_s) Dynamic_Equation(t,X_s,Struct_Param),...
    tspan,[X0 ; Xdot0],odeset('OutputFcn','odeplot')) ;

%% Ploting the Results

t=t' ; X=X'; y=X';
figure(1) ;
subplot(3,1,1) ;
plot(t,X(1,:));
ylabel('x') ;
grid ;
subplot(3,1,2) ;
plot(t,X(2,:)) ;
ylabel('y') ;
grid ;
subplot(3,1,3) ;
plot(t,X(3,:)) ;
ylabel('z') ;
grid ;
figure(2) ;
subplot(3,1,1) ;
plot(t,X(4,:)) ;
ylabel('rx') ;
grid ;
subplot(3,1,2) ;
plot(t,X(5,:)) ;
ylabel('ry') ;
grid ;
subplot(3,1,3) ;
plot(t,X(6,:)) ;
ylabel('rz') ;
grid ;

figure(3)
subplot(211)
plot(t,y(:,3),t,y(:,2),'--',t,y(:,1),'-.'),grid on
xlabel('time(sec)')
ylabel('positions m')
legend('z','y', 'x','location','Best')

subplot(212)
plot(t,y(:,4)*180/pi,t,y(:,5)*180/pi,'--',t,y(:,6)*180/pi,'-.'),grid on
xlabel('time(sec)')
ylabel('Orientation degrees')
axis([ts tf -0.5 1])
legend('\theta_x','\theta_y', '\theta_z','location','Best')
end