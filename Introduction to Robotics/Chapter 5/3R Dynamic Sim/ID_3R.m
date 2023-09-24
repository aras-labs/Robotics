%% 
% 	An Introduction to Robotics, 
%   By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
%   Copyright ARAS @2023
%
%   This function generates the  the dynamic behavior of 3R Robot
%   at configuration q
%
function xp = ID_3R(t,x)
% Robot Parameter initilization
run("Parameters.m");
q=x(1:3); dq=x(4:6);

%% Explicit Dynamic
   M=M_Matrix(q);
   G=G_Vector(q);
   C=C_Matrix(q,dq);
   
%% Torue inputs
tau=[0;0;0];
taud=[0;0;0];

%%  Differential Equation in state space form
xp=[dq;
    pinv(M)*(tau+taud-C*dq-G)];

   