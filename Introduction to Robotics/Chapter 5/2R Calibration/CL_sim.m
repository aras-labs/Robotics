%% 
% 	An Introduction to Robotics, 
%   By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
%   Copyright ARAS @2023
%
%   This code Runs the closed loop simulation of 2R manipulator
%   with PD controller
%   This simulation is executed for 12 different trajectories
%   and the results are saved in test.mat for Calibration Purpose
%   
%% Parameter Definitions
clear all; clf;
% Robot Parameter initilization
run("Parameters.m");
q0=Par.q0';
dq0=Par.dq0';
q0=q0(1:2);
x0=[q0 dq0];

% Solves DE by ode
[t,x] = ode23s(@PD_2R,tspan,x0, ...
    odeset('OutputFcn','odeplot','OutputSel',[1:2]));

%
%   Calculate the other system variables
%
q=x(:,1:2); dq=x(:,3:4);
N=max(size(t));
for i=1:N,
%----------------------------------------------------------------
%   The desired trajectory of the end effector 
%
[iqd(:,i),qd(:,i),dqd(:,i),d2qd(:,i)]=TP_cubic(t(i));
tau(i,:)=Par.KP*(qd(1:2,i)-q(i,:)')+Par.KV*(dqd(1:2,i)-dq(i,:)');
% 
%   Control Effort
end
iqd=iqd';qd=qd';dqd=dqd';d2qd=d2qd';
qd=qd(:,1:2);dqd=dqd(:,1:2);d2qd=d2qd(:,1:2);
eq=(q)*rad2deg;
%%
%       Linear Regression &
%       Least square solution 

% use simple diff for now
ddq=diff(dq)./diff(t);
ddq=[0 0;
    ddq];
Y1=[ddq(:,1),dq(:,1),cos(q(:,1))];
tau1=tau(:,1);
phi1=pinv(Y1)*tau1;
Y2=[ddq(:,2),dq(:,1),cos(q(:,1)+q(:,2))];
tau2=tau(:,2);
phi2=pinv(Y2)*tau2;
Phi=1e-3*[phi1; phi2]



%%  Simulated joint trajectories and actuator torques in closed loop
%   A sample figre is given in Figure 5.33 of the book
figure(1)
subplot(1,2,1)
plot(t,eq(:,1),'b',t,eq(:,2),'--k'),grid on
xlabel('time(sec)')
ylabel('joint motion (deg)')
legend('q_1','q_2','location','best')
subplot(1,2,2)
plot(t,tau(:,1),'b',t,tau(:,2),'--k'),grid on
xlabel('time(sec)')
ylabel('control effort (N.m)')
legend('\tau_1','\tau_2','location','best')
set(findall(figure(1),'type','line'),'linewidth',2)