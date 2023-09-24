%% 
% 	An Introduction to Robotics, 
%   By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
%   Copyright ARAS @2023
%
%   This code provides Dynamics formulation of 3R robot
%   Problems 5.3 and 5.7 (Christoffel Matrix)

%% Parameter Definitions
clear all, clc

% Defining symbolic variables:
n = 3; % Number of joints
th = sym('th',[n 1]);assume(th,'real'); % Joint angles
dth = sym('dth',[n 1]);assume(dth,'real'); % Joint angular velocities
a = sym('a',[n 1]);assume(a,'real'); % Link lengths
m = sym('m',[n 1]); assume(m,'real'); % Link masses
I = sym('I',[n 1]);assume(I,'real'); % Link moments of inertia
g = sym('g'); assume(g,'real'); % Gravitational acceleration
tau = sym('tau',[n 1]);assume(tau,'real'); % Joint torques

% Define moments of inertia for each link
I1 = m(1)*a(1)^2/12;
I2 = m(2)*a(2)^2/12;
I3 = m(3)*a(3)^2/12;

% Defining parametric variables:
z = [0;0;1]; % Unit vector along the z-axis
G = g*[0;-1;0];   % Gravitational force along -y direction
z1 = z; z2 = z; z3 = z;

% Define the positions of the centers of mass for each link
pc1 = sym('pc1',[3 n-2]);
pc2 = sym('pc2',[3 n-1]);
pc3 = sym('pc3',[3 n]);

% Calculate the positions of the centers of mass for each link
pc1(:,1) = 0.5*a(1)*[cos(th(1)); sin(th(1)); 0];
pc2(:,2) = 0.5*a(2)*[cos(th(1)+th(2)); sin(th(1)+th(2)); 0];
pc2(:,1) = [a(1)*cos(th(1)) + 0.5*a(2)*cos(th(1)+th(2));
            a(1)*sin(th(1)) + 0.5*a(2)*sin(th(1)+th(2)); 
            0];
pc3(:,3) = 0.5*a(3)*[cos(th(1)+th(2)+th(3)); sin(th(1)+th(2)+th(3)); 0];
pc3(:,2) = [a(2)*cos(th(1)+th(2)) + 0.5*a(3)*cos(th(1)+th(2)+th(3));
            a(2)*sin(th(1)+th(2)) + 0.5*a(3)*sin(th(1)+th(2)+th(3)); 
            0];
pc3(:,1) = [a(1)*cos(th(1)) + a(2)*cos(th(1)+th(2)) + 0.5*a(3)*cos(th(1)+th(2)+th(3));
            a(1)*sin(th(1)) + a(2)*sin(th(1)+th(2)) + 0.5*a(3)*sin(th(1)+th(2)+th(3)); 
            0];

% Define the Jacobian matrices for linear velocity and angular velocity
Jv1 = [cross(z1,pc1(:,1)) 0*z 0*z];
Jv2 = [cross(z1,pc2(:,1)) cross(z2,pc2(:,2)) 0*z];
Jv3 = [cross(z1,pc3(:,1)) cross(z2,pc3(:,2)) cross(z3,pc3(:,3))];
Jw1 = [z1 0*z 0*z];
Jw2 = [z1 z2 0*z];
Jw3 = [z1 z2 z3];

%% Lagrange Formulation
% Mass Matrix
disp('Mass matrix of 3R robot (Lagrange method):')
% Calculate the mass matrix for the robot
M = simplify(m(1)*Jv1'*Jv1 + Jw1'*I1*Jw1 + ...
    m(2)*Jv2'*Jv2 + Jw2'*I2*Jw2 + ...
    m(3)*Jv3'*Jv3 + Jw3'*I3*Jw3)

% Gravity Vector
disp('Gravity Vector of 3R robot (Lagrange method):')
% Calculate the gravity vector for the robot
P = simplify(-m(1)*G'*pc1(:,1) - m(2)*G'*pc2(:,1) - m(3)*G'*pc3(:,1));
G = simplify(jacobian(P,th)')

% Coriolis and centrifugal vector
disp('Coriolis and centrifugal Vector of 3R robot (Lagrange method):')
% Calculate the Coriolis and centrifugal vector for the robot
c1 = simplify([jacobian(M(:,1),th)*dth, jacobian(M(:,2),th)*dth , jacobian(M(:,3),th)*dth]*dth);
c2 = simplify(jacobian(dth'*M*dth,th)')/2;
c = simplify(c1 - c2)

% Christoffel Matrix
dM = [simplify(jacobian(M(1,:),th(1))), ...
    simplify(jacobian(M(2,:),th(1))), ...
    simplify(jacobian(M(3,:),th(1)));
    simplify(jacobian(M(1,:),th(2))), ...
    simplify(jacobian(M(2,:),th(2))), ...
    simplify(jacobian(M(3,:),th(2)));
    simplify(jacobian(M(1,:),th(3))), ...
    simplify(jacobian(M(2,:),th(3))), ...
    simplify(jacobian(M(3,:),th(3))); 
    ];
V = simplify(dM'*kron(eye(3,3),dth));
U = simplify(kron(eye(3,3),dth')*dM);
disp('Christoffel Matrix of 3R robot (Lagrange method):')
C = simplify((V+U'-U)/2)

%% Verification
disp('Check if c is equal to C*dq:')
Ec = simplify(c - C*dth)

% Check if V-2C is skew-symmetric
disp('Check if V-2C is skew-symmetric:')
MdC = simplify(V - 2*C)