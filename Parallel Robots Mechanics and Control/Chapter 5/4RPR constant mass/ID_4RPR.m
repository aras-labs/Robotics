%% 
%   Parallel Robots: Mechanics and Control
%   Copyright Hamid D. Taghirad 2013
%
%   This program generates the implicit Newton-Euler formulation of 
%   the planar cable manipulator.
%
%%
function res = ID_4RPR(t,y,yp)
% initialization
global Par
% Define Numeric Variables
Ath=Par.Ath;      Bth=Par.Bth;      
% parameter perturbation
M=Par.pert*Par.M;
I=Par.pert*Par.I;
rho=Par.pert*Par.rho;
RA=Par.pert*Par.RA;
RB=Par.pert*Par.RB;
% Force and gravity parameters
F=Par.F;
Fd=Par.Fd;
g=Par.g;
% state assignments
x=y(1:3);
dx=y(4:6);
d2x=yp(4:6);
phi=x(3);
dphi=dx(3);
K=[0,0,1]';
%
%   provide the inverse dynamics formulation
%
for i=1:4
% Kinematics
A(:,i)=RA*[cos(Ath(i)); sin(Ath(i))];
E(:,i)=RB*[cos(Bth(i)+phi); sin(Bth(i)+phi)];
X(i,1)=x(1)-A(1,i)+ E(1,i);
Y(i,1)=x(2)-A(2,i)+ E(2,i);

al(i,1)=atan2(Y(i),X(i));
L(i,1)=sqrt(X(i)^2+Y(i)^2);
S(1,i)=cos(al(i));
S(2,i)=sin(al(i));
SS(:,i)=[S(:,i);0];
N(:,i)=cross(K,SS(:,i));
gn(i)=dot(g,N(:,i));
gs(i)=dot(g,SS(:,i));

% Jacobians
Jx(i,1)=S(1,i);
Jx(i,2)=S(2,i);
Jx(i,3)=E(1,i)*S(2,i)-E(2,i)*S(1,i);
dL(i,1)=Jx(i,:)*dx;


Ja(i,1)=-S(2,i);
Ja(i,2)=S(1,i);
Ja(i,3)=E(1,i)*S(1,i)+E(2,i)*S(2,i);
dal(i,1)=Ja(i,:)*dx/L(i);
% Accelerations
d2L(i,1)=Jx(i,:)*d2x- ... 
    dx(3)^2*(E(1,i)*S(1,i)+E(2,i)*S(2,i))+ dal(i)^2*L(i);
 
d2al(i,1)=1/L(i)*(Ja(i,:)*d2x - ...
    (E(2,i)*S(1,i)-E(1,i)*S(2,i))*dx(3)^2 - 2*dL(i)*dal(i));
% Reacting forces
Fn(i,1)=rho/6*(2*d2al(i)*L(i)^2+3*(L(i)*dL(i)*dal(i)-L(i)*gn(i)));
Fs(i,1)=F(i)+rho/2*(L(i)*d2L(i)-(dal(i)*L(i))^2-2*L(i)*gs(i));
end
%
% Newton-Euler equations for moving platform
%
V(1,1)=M*(d2x(1)-g(1));
V(2,1)=M*(d2x(2)-g(2));
V(3,1)=I*d2x(3);
for i=1:4
V(1,1)= V(1,1)+ Fs(i)*S(1,i)-Fn(i)*S(2,i);
V(2,1)= V(2,1)+Fs(i)*S(2,i)+Fn(i)*S(1,i);
V(3,1)= V(3,1)+ Fs(i)*(E(2,i)*S(1,i)-E(1,i)*S(2,i)) - ...
                    Fn(i)*(E(1,i)*S(1,i)+E(2,i)*S(2,i));
end

%   The implicit ode is augmented from the first three derivatives and the
%   second three equations of motion

res = [ yp(1:3)- dx;    % These three equation are the simple derivative dx=v
        V - Fd ];       % These three equation are the main inverse dynamics equations
    