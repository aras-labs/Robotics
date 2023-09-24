%% 
% 	An Introduction to Robotics, 
%   By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
%   Copyright ARAS @2023
%
%   This code provides Dynamics formulation of 2R robot
%   Problems 5.1 and 5.6 (Christoffel Matrix)
%
%% Parameter Definitions
clear all, clc

%   Defining symbolic variables:
n=2;
syms g; assume(g,'real');
th=sym('th',[n 1]);assume(th,'real');
dth=sym('dth',[n 1]);assume(dth,'real');
a=sym('a',[n 1]);assume(a,'real');
m=sym('m',[n 1]); assume(m,'real');
I=sym('I',[n 1]);assume(I,'real');
tau=sym('tau',[n 1]);assume(tau,'real');
I1=m(1)*a(1)^2/12;
I2=m(2)*a(2)^2/12;

%   Defining parametric variables:
z=[0;0;1];
G=g*[0;-1;0];   % g along -y direction
z1=z;z2=z;

% link's center of mass
pc1=sym('pc1',[3 n-1]);
pc2=sym('pc1',[3 n]);
pc1(:,1)=0.5*a(1)*[cos(th(1)); sin(th(1)); 0];
pc2(:,2)=0.5*a(2)*[cos(th(1)+th(2)); sin(th(1)+th(2)); 0];
pc2(:,1)=[a(1)*cos(th(1)) + 0.5*a(2)*cos(th(1)+th(2));
         a(1)*sin(th(1)) + 0.5*a(2)*sin(th(1)+th(2)); 
         0];

% link's Jacobian matrices
Jv1=[cross(z1,pc1(:,1)) 0*z];
Jv2=[cross(z1,pc2(:,1)) cross(z2,pc2(:,2))];
Jw1=[z1 0*z]; Jw2=[z1 z2];

%%   Lagrange Formulation

%   Mass Matrix
disp('Mass matrix of 2R robot (Lagrange method):')
M= simplify( m(1)*Jv1'*Jv1+Jw1'*I1*Jw1 + ...
    m(2)*Jv2'*Jv2+Jw2'*I2*Jw2)

%   Gravity Vector
disp('Gravity vector of 2R robot (Lagrange method):')
P=simplify(-m(1)*G'*pc1(:,1) -m(2)*G'*pc2(:,1));
G=simplify(jacobian(P,th)')

%    Coriolis and centrifugal vector
disp('oriolis and centrifugal vector of 2R robot (Lagrange method):')
c1=simplify([jacobian(M(:,1),th)*dth, jacobian(M(:,2),th)*dth]*dth);
c2=simplify(jacobian(dth'*M*dth,th)')/2;
c=simplify(c1-c2)

% Christoffel Matrix
dM=[simplify(jacobian(M(1,:),th(1))), simplify(jacobian(M(2,:),th(1)));
    simplify(jacobian(M(1,:),th(2))), simplify(jacobian(M(2,:),th(2)))];
V=simplify(dM'*kron(eye(2,2),dth));
U=simplify(kron(eye(2,2),dth')*dM);
disp('Christoffel Matrix of 2R robot (Lagrange method):')
C=simplify((V+U'-U)/2)

%% Verification
% Check if c is equal to C\dot{q}
disp('Check if c is equal to C\dot{q}:')
Ec=simplify(c-C*dth)

% Check if \dot{m}-2C is skew-symmetric
disp('Check if \dot{m}-2C is skew-symmetric:')
MdC=simplify(V-2*C)

