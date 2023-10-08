%% 
%   Parallel Robots: Mechanics and Control
%   Copyright Hamid D. Taghirad 2013
%
%   This program verifies the inverse and forward kinematic analysis of
%   Hydraulic Shoulder Manipulator.
%
%%      
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

Par.lp=0.25;
Par.lb=0.2;
Par.ld=0.05;
Par.lk=0.01;
Par.alpha=30*deg2rad;
Par.time=0; 
Par.index=0;
Par.threshold=1;
%
%   The desired trajectory initial points
%     Time    th1           th2             th3 
Par.xd=[0     30*deg2rad    0*deg2rad       0*deg2rad
        1     0*deg2rad     30*deg2rad     -30*deg2rad
        2     30*deg2rad    0*deg2rad       0*deg2rad
        3     0*deg2rad     0*deg2rad       0*deg2rad
        10    0*deg2rad     0*deg2rad       0*deg2rad
        ];
lp=Par.lp; lb=Par.lb; ld=Par.ld; lk=Par.lk;
alpha=Par.alpha; sa=sin(alpha);ca=cos(alpha);

k(1)=lk^2+lb^2+ld^2+4*lp^2-4*lp*lk;
k(2)=2*lb*(lk-2*lp)*sa;
k(3)=2*lb*(2*lp-lk)*ca;
k(4)=-2*lb*ld*sa;
k(5)=-2*lb*ld*ca;
Par.k=k;
th_old = Par.xd(1,2:4)';        % found from first iteration
options=optimset('Display','off');
%%  Execute the subroutines
%   
%-------------------- Start iteration ------------------------ 
for i=1:N;
    t(i)=dt*(i-1);
    Par.time=t(i);
    thd(:,i)=TP_cubic(t(i));
%----------------------------------------------------------------
%   Solve the inverse kinematics
%
    L(:,i)=InvKin_HSM(thd(:,i));
%----------------------------------------------------------------    
%   Solve the Forward kinematics
%
      th(:,i)=FK_HSM(L(:,i),th_old);
      th_old=th(:,i); 
end

%% plots the result
%
figure(1); clf
plot(t,L(1,:),t,L(2,:),'--',t,L(3,:),'-.',t,L(4,:),'k:'),grid
ylabel('Limb lengths (m)'),xlabel('time(sec)')
legend('L_1','L_2','L_3','L_4','Location','Best');

figure(2); clf
plot(t,th(1,:)*rad2deg,t,th(2,:)*rad2deg,'--',...
    t,th(3,:)*rad2deg,'-.'),grid
ylabel('Euler Angles (degree)'),xlabel('time(sec)')
legend('\theta_1','\theta_2','\theta_3','Location','Best');

figure(3); clf
subplot(211)
plot(t,thd(1,:)-th(1,:),'x',t,thd(2,:)-th(2,:),'+',t,thd(3,:)-th(3,:),'o'),grid
xlabel('time(sec)')
title('\theta_d - \theta_c: Forward Kinematics Verification')
legend('e_{\theta_1}','e_{\theta_2}','e_{\theta_3}');
