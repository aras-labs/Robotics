%% 
%   Parallel Robots: Mechanics and Control
%   Copyright Hamid D. Taghirad 2013
%
%   This program executes the inverse and forward kinematics 
%   of the planar cable manipulator and verifies the results
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
t0=0; tf=10;
tspan=[t0 tf];              % simulation time
% The orientation of Ai, Bi, ai, and bi's in its at central position
Par.Ath=[-135;-45;45;135]*deg2rad; 
Par.Bth=[-45;-135;135;45]*deg2rad;
Par.RA=900;                  % the Ai's circle radius 
Par.RB=10;                   % the Bi's circle radius 

%
%   The desired trajectory initial points
%
%       Time    x       y       \phi       (in SI units)
Par.xd=[0       0       0       0
        10      200    -100     pi/4
        200     0       0       0
        300     0       0       0
        400     0       0       0
        500     0       0       0
        600     0       0       0
        1000    0       0       0];
    
%   Evaluate parameters used in the main from global Par structure
RA = Par.RA; RB = Par.RB;                 
Ath=Par.Ath; Bth=Par.Bth;
%
%   the initial values for wrappings
alpha_old = [45;135;-135;45]*deg2rad; % found from first iteration
x_old     = zeros(4,1);    % the initial position and orientation
A=[RA*cos(Ath) RA*sin(Ath)]';

%%  Execute the subroutines
%   

dt=0.05;
Tf=10;
N=Tf/dt;
for i=1:N+1
%----------------------------------------------------------------
%   The desired trajectory of the end effector 
%
 t(i)=dt*(i-1);             % Time array
 [ixd(:,i),xd(:,i),dxd(:,i),d2xd(:,i)]=TP_cubic(t(i));
%----------------------------------------------------------------
%   Solve the inverse kinematics
%
    [L(:,i),alpha(:,i)]=InvKin_4RPR(xd(1:3,i),A,RB,Bth,alpha_old);
    alpha_old=alpha(:,i);   % next step initial guess
%----------------------------------------------------------------    
%   Solve the Forward kinematics
%
    xc(:,i)=FK_4RPR(L(:,i),A,RB,Bth,x_old);
    x_old=xc(:,i);          % next step initial guess

end

%% plots the result
%
figure(1)
subplot(211)
plot(t,xd(1,:),t,xd(2,:),'-.'),grid
title('A Typical Trajectory for 4RPR Manipulator')
xlabel('time(sec)')
ylabel('x and y (m)')
legend('x','y','location','best')
subplot(212)
plot(t,xd(3,:)*rad2deg),grid
xlabel('time(sec)')
ylabel('\phi (degrees)')
legend('\phi','location','best')

figure(2)
clf
subplot(211)
plot(t,L(1,:),t,L(4,:),'-.'),grid
title('Manipulator Limb lengths')
xlabel('time(sec)')
ylabel('Limb lengths (m)')
legend('L_1','L_4','location','best')
subplot(212)
plot(t,L(2,:),t,L(3,:),'-.'),grid
xlabel('time(sec)')
ylabel('Limb lengths (m)')
legend('L_2','L_3','location','best')

figure(3)
clf
subplot(211)
plot(t,xd(1,:)-xc(1,:),'x',t,xd(2,:)-xc(2,:),'+',t,xd(3,:)-xc(3,:),'o'),grid
title('x_d-x_c: Forward Kinematics Verification')
xlabel('Time(sec)')
legend('e_x','e_y','e_\phi')