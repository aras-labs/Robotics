%% 
%   Robotics Course, Professor: Prof. Hamid D. Taghirad
%   Aras.kntu.ac.ir/education/robotics
%   Copyright 2020
%
%   This code provides the Isortrpic analysis of 2R manipulator
%%
clear;
clc;

syms th2 real
syms x

a1=sqrt(2); a2=1; k=10^3;
J=[a1*sin(th2)  0; a1*cos(th2)+a2 a2];
JJ=simplify(J*J');
p=x*eye(2,2)-JJ;
D=simplify(det(p));
lambda=simplify(solve(D==0,x));

J1=[sqrt(2) , 0; 1 1];
JJ1=J1*J1'; 
[U,S,V]=svd(J1);
[v1,e1]=eig(JJ1);

C=J1*J1'/k;
[U,S,V]=svd(C)

[v,e]=eig(C'*C)

% th2=0:0.01:pi;
% alpha=[0.707; 1; 1.2; 1.8];
% for i=1:4;
% kappa(:,i)=2*sin(th2)./(1/alpha(i)+2*alpha(i)+2*cos(th2));
% end
% plot(th2/pi,kappa(:,1),'-k', th2/pi,kappa(:,2),'--', ... 
% th2/pi,kappa(:,3),'-.', th2/pi,kappa(:,4),'m:')
% set(findall(figure(1),'type','line'),'linewidth',2)
% grid on
% legend('\alpha=0.707',  '\alpha=1, 0.5', '\alpha=1.2, 0.417', ...
%     '\alpha=1.8, 0.278')
% xlabel('\theta_2/\pi')
% ylabel('\kappa')

