%   Copyright Hamid D. Taghirad 2022
%
%   This program simulates the closed loop 
%   dynamic behavior of 3R Robot with a PD Controller
% 
function xp = PD_3R(t,x)

% Robot Parameter initilization
run("Parameters.m");
q=x(1:3); dq=x(4:6);
[iqd,qd,dqd,d2qd]=TP_cubic(t);

%% Explicit Dynamic
   M=M_Matrix(q);
   G=G_Vector(q);
   C=C_Matrix(q,dq);
   
%% Torue inputs PD control
tau=Par.KP*(qd-q)+Par.KV*(dqd-dq);
taud=[0;0;0];

%%  Differential Equation in state space form
xp=[dq;
    pinv(M)*(tau+taud-C*dq-G)];

   