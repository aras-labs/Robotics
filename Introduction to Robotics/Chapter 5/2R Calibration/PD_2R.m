%% 
% 	An Introduction to Robotics, 
%   By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
%   Copyright ARAS @2023
%
%   This function generates the closed loop function of the 2R manipulator
%   with PD controller
function xp = PD_2R(t,x)

run("Parameters.m");
q=x(1:2); dq=x(3:4);
[iqd,qd,dqd,d2qd]=TP_cubic(t);

%% Explicit Dynamic + Actuator Dynamics
   M=M_Matrix(q)+diag(Par.eta.^2.*Par.Im);
   G=G_Vector(q);
   C=C_Matrix(q,dq)+diag(Par.b+Par.bm.*Par.eta.^2);
   
%% Torue inputs PD control
tau=Par.KP*(qd(1:2)-q)+Par.KV*(dqd(1:2)-dq);
taud=[0;0];

%%  Differential Equation in state space form
xp=[dq;
    pinv(M)*(tau+taud-C*dq-G)];

   