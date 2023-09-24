%% 
% 	An Introduction to Robotics, 
%   By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
%   Copyright ARAS @2023
%
%   This program generates the measurement noise 
%
clf, clc
clear all
deg2rad = pi/180 ;
rad2deg = 180/pi;

%       Time    q1             q2            q3        
SP.xd=[
        0    00*deg2rad    00*deg2rad   00*deg2rad
        2    60*deg2rad   -30*deg2rad   30*deg2rad
        4    0*deg2rad      0*deg2rad   0*deg2rad
        6    0*deg2rad      0*deg2rad   0*deg2rad
        8    0*deg2rad      0*deg2rad   0*deg2rad
        10   0*deg2rad      0*deg2rad   0*deg2rad
        ];

noise_q  = 5e-3*[1;1;1];  % noise
noise_dq = 1e-2*[1;1;1];


t=0:0.02:4;
N=max(length(t));

for i=1:N
    [xd,dxd,d2xd]=TP_quintic(t(i),SP);
    nq=noise_q*randn;
    ndq=noise_dq*randn;
    q(:,i)=xd;
    qn(:,i)=xd+nq;
    dq(:,i)=dxd;
    dqn(:,i)=dxd+ndq;
    d2q(:,i)=d2xd;
end

figure(1)
plot(t,qn(1,:),'k', t, qn(2,:),'-.k', t, qn(3,:),'--k'),grid,
legend('qx','qy','qz')
  set(findall(figure(1),'type','line'),'linewidth',2)
figure(2)
plot(t,dqn(1,:),'k', t, dqn(2,:),'-.k', t, dqn(3,:),'--k'),grid,
legend('qx','qy','qz')
  set(findall(figure(2),'type','line'),'linewidth',2)

qnoise=[t' qn(1,:)' dqn(1,:)'];

