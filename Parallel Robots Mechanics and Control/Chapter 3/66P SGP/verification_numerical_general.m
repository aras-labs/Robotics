%% 
%   Parallel Robots: Mechanics and Control
%   Copyright Hamid D. Taghirad 2013
%
%   This program verifies the inverse and forward kinematic analysis of
%   general Stewart--Gough platform numerical solution
%
%%
%      
%   Initialization
%
clear all;

global Par      % Use all parameters in a Global Structure
%   initialize the simulation parameters
deg2rad=pi/180;
rad2deg=180/pi;
dt=0.01; Tf=2; N=Tf/dt;
% 
%   The coordinates of Ai's, Bi's, ai's and bi's at initial position
%   Consider the fixed coordinate is located at the center 
%   at initial position
Par.time=0; 
Par.index=0;
sx=1/sqrt(2);
%
%   The desired trajectory initial points
%     Time    x      y     z    s_x   s_y   s_z  \theta
Par.xd=[0     0      0     1     0     1     1    30*sqrt(2)*deg2rad
        1     1     1.5    1.5   1     0     1    60*sqrt(2)*deg2rad
        2     0      0     1     0     1     1    30*sqrt(2)*deg2rad
        3     0      0     0     1     1     1    0*deg2rad
        4     0      0     0     1     1     1    0*deg2rad
        5     0      0     0     1     1     1    0*deg2rad
        6     0      0     0     1     1     1    0*deg2rad
        10    0      0     0     1     1     1    0*deg2rad
        ];
A =[
    0.8000    0.5000   -0.2000   -0.6000   -0.4000    0.4000
         0    1.2000    1.0000    0.4000   -0.8000   -1.2000
%        0         0         0         0         0         0
        0.1     -0.2       -0.1      0.2       -0.1     +0.1
%       0.1         -0.1       -0.1        0.2       0      -0.1
];
B=[
    0.6000    0.3000   -0.1000   -0.4000   -0.2000    0.3000
    0.2000    1.0000    1.2000    0.2000   -1.0000   -1.0000
%       0         0         0         0         0         0
     -0.1         0.2      -0.2     -0.1       0.1     -0.1
%       -0.1         0.2       0.1        -0.1       0.1       0.1
   ];

Par.A=A;
Par.B=B;

x_old = Par.xd(1,2:8)';
options=optimset('Display','off', 'Largescale','off');

%%  Execute the subroutines
%
%-------------------- Start iteration ------------------------ 
for i=1:N;
    t(i)=dt*(i-1);
    [xd(:,i),sd(:,i),thd(:,i)]=TP_cubic_s(t(i));
    %sd(:,i)=sd(:,i)./norm(sd(:,i));    %normalize the screw axis
    RAB(:,:,i)=sc2rot(sd(:,i),thd(:,i));
%----------------------------------------------------------------
%   Solve the inverse kinematics
%
    D(:,i)=InvKin_SGP(xd(:,i),RAB(:,:,i));
%----------------------------------------------------------------    
%   Solve the Forward kinematics
%
    ttt=cputime;
    xc(:,i)=FK_SGP_pure(D(:,i),x_old);
    CPT(:,i)=cputime-ttt;
    x_old=xc(:,i); 
end
% return
Xd=[xd;sd;thd];

%% plots the result
%
figure(1);clf
subplot(211)
plot(t,xd(1,:),t,xd(2,:),'--',t,xd(3,:),'-.'),grid
xlabel('time(sec)')
ylabel('displacement (m)')
legend('x','y','z')
subplot(212)
plot(t,sd(1,:).*thd(1,:)*rad2deg,t,sd(2,:).*thd(1,:)*rad2deg,'--',...
    t,sd(3,:).*thd(1,:)*rad2deg,'-.'),grid
xlabel('time(sec)')
ylabel('Orientation Angels (degrees)')
legend('\theta_x','\theta_y','\theta_z')

figure(2);clf
subplot(211)
plot(t,D(1,:),t,D(2,:),'--',t,D(3,:),'-.'),grid
xlabel('time(sec)')
ylabel('Limb length (m)')
legend('l_1','l_2','l_3')
subplot(212)
plot(t,D(4,:),t,D(5,:),'--',t,D(6,:),'-.'),grid
xlabel('time(sec)')
ylabel('Limb length (m)')
legend('l_4','l_5','l_6')

figure(3);clf
subplot(211)
plot(t,Xd(1,:)-xc(1,:),'x',t,Xd(2,:)-xc(2,:),'+',t,Xd(3,:)-xc(3,:),'co'),grid
xlabel('time(sec)')
ylabel('Position error (m)')
legend('e_x','e_y','e_z')
subplot(212)
plot(t,Xd(7,:).*Xd(4,:)-xc(7,:).*xc(4,:),'x', ...
     t,Xd(7,:).*Xd(5,:)-xc(7,:).*xc(5,:),'+', ...
     t,Xd(7,:).*Xd(6,:)-xc(7,:).*xc(6,:),'co'),grid
xlabel('time(sec)')
ylabel('Orientation error')
legend('e_{\thetax}','e_{\thetay}','e_{\thetaz}')