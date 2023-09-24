%% 
% 	An Introduction to Robotics, 
%   By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
%   Copyright ARAS @2023
%
%   This code provides Dynamics formulation of 3DOF Elbow manipulator
%   Problems 5.5 and 5.9 (Christoffel Matrix)
%
%% Parameter Definitions
clear all, clc

%   Defining symbolic variables:
n=3;
n=3;
th=sym('th',[n 1]);assume(th,'real');
dth=sym('dth',[n 1]);assume(dth,'real');
a=sym('a',[n 1]);assume(a,'real');
m=sym('m',[n 1]); assume(m,'real');
I=sym('I',[n 1]);assume(I,'real');
g=sym('g'); assume(g,'real');
I_1=sym('I_1');assume(I_1,'real');
tau=sym('tau',[n 1]);assume(tau,'real');

T1=DH(0,pi/2,0,th(1));  T1=Matrix_Vpa(T1,4,4);
R1=T1(1:3,1:3);
T2=DH(a(2),0,0,th(2)); T2=Matrix_Vpa(T2,4,4);
R2=T2(1:3,1:3);
T3=DH(a(3),0,0,th(3)); T3=Matrix_Vpa(T3,4,4);
R3=T3(1:3,1:3);

R12=simplify(R1*R2);
R13=simplify(R12*R3);
R23=simplify(R2*R3);

I=[0 0 0; 0 0 0; 0 0 I_1];
II=[0 0 0; 0 1 0; 0 0 1];
II1=simplify(R1*I*R1');
II2=simplify(R12*II*R12');
II3=simplify(R13*II*R13');

I1=simplify(II1);
I2=simplify(m(2)*a(2)^2*II2/12);
I3=simplify(m(3)*a(3)^2*II3/12);

%   Defining parametric variables:
z=[0;0;1];
z1=z;
z2=R1*z;z3=R12*z;
%z1=[cos(th(1));sin(th(1));0];
bg=g*[0;-1;0];      % g along -y direction
threshold=1e-10;    % Threshold for small matrix element elimiation

pc1=sym('pc1',[3 n-2]);
pc2=sym('pc2',[3 n-1]);
pc3=sym('pc3',[3 n]);

% link's center of mass
pc1(:,1)=simplify(-0.5*a(1)*[0;0;1]);
%pc1(:,1)=0*[1;0;0];
pc2(:,2)=simplify(0.5*a(2)*R12*[1;0;0]);
pc2(:,1)=pc2(:,2);
pc3(:,3)=simplify(0.5*a(3)*R13*[1;0;0]);
pc3(:,2)=simplify(simplify(a(2)*R12*[1;0;0])+pc3(:,3));
pc3(:,1)=pc3(:,2);

% link's Jacobian matrices
Jv1=simplify([cross(z1,pc1(:,1)) 0*z 0*z]);
Jv2=simplify([cross(z1,pc2(:,1)) cross(z2,pc2(:,2)) 0*z]);
Jv3=simplify([cross(z1,pc3(:,1)) cross(z2,pc3(:,2)) cross(z3,pc3(:,3))]);

Jw1=[z1 0*z 0*z]; Jw2=[z1 z2 0*z]; Jw3=[z1 z2 z3];

%%   Lagrange Formulation
%   Mass Matrix

%M= simplify(m(1)*Jv1'*Jv1+Jw1'*I1*Jw1 + ...
%            m(2)*Jv2'*Jv2+Jw2'*I2*Jw2 + ...
%            m(3)*Jv3'*Jv3+Jw3'*I3*Jw3);
M1= simplify(m(1)*Jv1'*Jv1+ m(2)*Jv2'*Jv2+ m(3)*Jv3'*Jv3);
M2=simplify(Jw1'*I1*Jw1 + Jw2'*I2*Jw2 + Jw3'*I3*Jw3);
%M1=Matrix_Vpa(M1,3,3);
%M2=Matrix_Vpa(M2,3,3);
M=simplify(M1+M2);
%   Eliminate small matrix components by vpareal function
disp('Mass matrix of Elow manipulator (Lagrange method):')
M=mapSymType(M, 'vpareal', @(x) piecewise(abs(x)<=threshold, 0, x))

%   Gravity Vector
P=simplify(-m(1)*bg'*pc1(:,1) -m(2)*bg'*pc2(:,1) -m(3)*bg'*pc3(:,1));
%   Using Potential Energy derivative for G derivation
G=simplify(jacobian(P,th)');
disp('Gravity Vector of Elow manipulator (Lagrange method):')
%   Using link's jacobians for G derivation
G1=simplify(-m(1)*Jv1'*bg-m(2)*Jv2'*bg-m(3)*Jv3'*bg)

%   Verification
disp('Verification of Gravity Vector of Elow manipulator (Lagrange method):')

simplify(G-G1)

%    Coriolis and centrifugal vector
c1=simplify([jacobian(M(:,1),th)*dth, jacobian(M(:,2),th)*dth , ...
                                        jacobian(M(:,3),th)*dth]*dth);
c2=simplify(jacobian(dth'*M*dth,th)')/2;
disp('Coriolis and centrifugal Vector of Elow manipulator (Lagrange method):')
c=simplify(c1-c2);
%   Eliminate small matrix components by vpareal function
c=mapSymType(c1, 'vpareal', @(x) piecewise(abs(x)<=threshold, 0, x))

% Christoffel Matrix
dM=[simplify(jacobian(M(1,:),th(1))), ...
    simplify(jacobian(M(2,:),th(1))), ...
    simplify(jacobian(M(3,:),th(1)));
    simplify(jacobian(M(1,:),th(2))), ...
    simplify(jacobian(M(2,:),th(2))), ...
    simplify(jacobian(M(3,:),th(2)));
    simplify(jacobian(M(1,:),th(3))), ...
    simplify(jacobian(M(2,:),th(3))), ...
    simplify(jacobian(M(3,:),th(3))); 
    ];
V=simplify(dM'*kron(eye(3,3),dth));
U=simplify(kron(eye(3,3),dth')*dM);
disp('Christoffel matrix of Elow manipulator (Lagrange method):')
C=simplify((V+U'-U)/2);
%   Eliminate small matrix components by vpareal function
C=mapSymType(C, 'vpareal', @(x) piecewise(abs(x)<=threshold, 0, x))

%% Verification
% Check if \dot{m}-2C is skew-symmetric
disp('Check if \dot{m}-2C is skew-symmetric:')
D=simplify(V-2*C);
D=mapSymType(D, 'vpareal', @(x) piecewise(abs(x)<=threshold, 0, x))