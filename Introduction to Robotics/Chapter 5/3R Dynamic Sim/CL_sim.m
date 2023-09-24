%% 
% 	An Introduction to Robotics, 
%   By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
%   Copyright ARAS @2023
%
%   This code Runs the closed loop simulation of 3R manipulator
%   with PD controller
%   Figures 5.15 of the book
%
%   
%% Parameter Definitions

clear all; clc, clf;
% Robot Parameter initilization
run("Parameters.m");
q0=Par.q0';
dq0=Par.dq0';
x0=[q0 dq0];

% Solves DE by ode
[t,x] = ode23s(@PD_3R,tspan,x0, ...
    odeset('OutputFcn','odeplot','OutputSel',[1:3]));

%
%   Calculate the other system variables
%
q=x(:,1:3); dq=x(:,4:6);
N=max(size(t));
for i=1:N,
%----------------------------------------------------------------
%   The desired trajectory of the end effector 
%
[iqd(:,i),qd(:,i),dqd(:,i),d2qd(:,i)]=TP_cubic(t(i));
tau(i,:)=Par.KP*(qd(:,i)-q(i,:)')+Par.KV*(dqd(:,i)-dq(i,:)');
% 
%   Control Effort
end
iqd=iqd';qd=qd';dqd=dqd';d2qd=d2qd';
eq=(qd-q)*rad2deg;


%%  Simulated joint trajectories and actuator torques in closed loop
%   Figure 5.15 of the book
figure(1)
subplot(1,2,1)
plot(t,eq(:,1),'b',t,eq(:,2),'--k',t,eq(:,3),'-.r'),grid on
xlabel('time(sec)')
ylabel('tracking errors (deg)')
legend('e_{q_1}','e_{q_2}','e_{q_3}','location','best')
subplot(1,2,2)
plot(t,tau(:,1),'b',t,tau(:,2),'--k',t,tau(:,3),'-.r'),grid on
xlabel('time(sec)')
ylabel('control effort (N.m)')
legend('\tau_1','\tau_2','\tau_3','location','best')
set(findall(figure(1),'type','line'),'linewidth',2)