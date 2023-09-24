%% 
% 	An Introduction to Robotics, 
%   By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
%   Copyright ARAS @2023
%
%   This code Runs the  Inverse dynamics of 3R manipulator
%   with PD controller
%
%   Figures 5.13 and 5.14
%
%% Parameter Definitions

clear all;
% Robot Parameter initilization
run("Parameters.m");

t=t0:0.01:tf;
N=max(size(t));
for i=1:N
%----------------------------------------------------------------
%   The desired trajectory of the joints
%
[iq(:,i),q(:,i),dq(:,i),d2q(:,i)]=TP_cubic(t(i));
%----------------------------------------------------------------
% Dynamic Parameters
%
M=M_Matrix(q(:,i));
G=G_Vector(q(:,i));
C=C_Matrix(q(:,i),dq(:,i));
%----------------------------------------------------------------
% FD torques to jenerate the trajectories
%
tau(:,i)=M*d2q(:,i)+C*dq(:,i)+G;
end
iq=iq';q=q';dq=dq';d2q=d2q';
tau=tau';
q=q*rad2deg;

%%  Joint trajectories and actuator torques
%   Figure 5.13 of the book
figure(1)
subplot(1,2,1)
plot(t,q(:,1),'b',t,q(:,2),'--k',t,q(:,3),'-.r'),grid on
xlabel('time(sec)')
ylabel('joint angles (deg)')
legend('q_1','q_2','q_3','location','best')
subplot(1,2,2)
plot(t,tau(:,1),'b',t,tau(:,2),'--k',t,tau(:,3),'-.r'),grid on
xlabel('time(sec)')
ylabel('joint torques (N.m)')
legend('\tau_1','\tau_2','\tau_3','location','best')
set(findall(figure(1),'type','line'),'linewidth',2)


%%  Power Plots for the 3R robot
%   Figure 5.14 of the book
figure(2)
subplot(1,2,1)
plot(t,tau(:,1).*dq(:,1),'b',t,tau(:,2).*dq(:,2),'--k',...
    t,tau(:,3).*dq(:,3),'-.r'),grid on
xlabel('time(sec)')
ylabel('power (Watts)')
legend('j_1','j_2','j_3','location','best')
subplot(1,2,2)
plot(dq(:,2),tau(:,1),'b',dq(:,2),tau(:,2),'--k',...
    dq(:,3),tau(:,3),'-.r'),grid on
xlabel('angular velocity (rad/sec)')
ylabel('joint torques (N.m)')
set(findall(figure(2),'type','line'),'linewidth',2)
