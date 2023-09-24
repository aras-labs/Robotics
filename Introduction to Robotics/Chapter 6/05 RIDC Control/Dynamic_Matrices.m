%% 
% 	An Introduction to Robotics, 
%   By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
%   Copyright ARAS @2023
%
%   This function generates the dynamic matrices 
%   of the 3R planar robot

function [Dynamic_Mats] = Dynamic_Matrices(qs, SP)
% Dynamic_Matrices computes the dynamic matrices (Mass matrix, Coriolis/centrifugal matrix, and gravity vector) of a robot.
%
%   [Dynamic_Mats] = Dynamic_Matrices(qs, SP)
%
%   Inputs:
%   - qs: Joint variables and their derivatives (position and velocity).
%   - SP: Robot parameters (including lengths, masses, and gravitational acceleration).
%
%   Outputs:
%   - Dynamic_Mats: Struct containing the dynamic matrices (M, C, G) and their perturbed versions (Mh, Ch, Gh).

% Extract joint variables and their derivatives
q  = qs(1:SP.n); % Joint positions
dq = qs(SP.n+1:2*SP.n); % Joint velocities

% Parameters initialization
a1 = SP.a(1); a2 = SP.a(2); a3 = SP.a(3); % Link lengths
m1 = SP.m(1); m2 = SP.m(2); m3 = SP.m(3); % Link masses
g = SP.g; % Gravitational acceleration

% Extract joint positions and velocities
q1 = q(1); q2 = q(2); q3 = q(3);
dq1 = dq(1); dq2 = dq(2); dq3 = dq(3);
c1 = cos(q1); s1 = sin(q1);
c2 = cos(q2); s2 = sin(q2);
c3 = cos(q3); s3 = sin(q3);
c12 = cos(q1 + q2); s12 = sin(q1 + q2);
c23 = cos(q2 + q3); s23 = sin(q2 + q3);
c123 = cos(q1 + q2 + q3); s123 = sin(q1 + q2 + q3);

% Mass matrix (M) computation
M11 = (m1/3 + m2 + m3) * a1^2 + (m2/3 + m3) * a2^2 + m3 * a3^2 + ...
    (m2 + 2 * m3) * a1 * a2 * c2 + a3 * m3 * (a1 * c23 + a2 * c3);
M12 = (m2/3 + m3) * a2^2 + m3 * a3 * (a3/3 + a2 * c23/2) + ...
    m2/2 * a1 * a2 * c2 + m3 * a2 * (a1 * c2 + a3 * c23);
M13 = m3 * a3 * (a3/3 + a2 * c23/2 + a2 * c3/2);
M22 = (m2/3 + m3) * a2^2 + m3 * a3^2/3 + m3 * a2 * a3 * c3;
M23 = m3 * a3 * (a3/3 + a2 * c3/2);
M33 = m3 * a3^2/3;

Dynamic_Mats.M = ...
    [M11 M12 M13;
     M12 M22 M23;
     M13 M23 M33];

% Coriolis/centrifugal matrix (C) computation
C11 = 0;
C12 = -a1 * (a2 * s2 * m2 + (2 * a2 * s2 + a3 * s23) * m3) * (dq1 + dq2/2) - a1/2 * a3 * s23 * m3 * dq3;
C13 = -a3 * (a1 * s23 + a2 + s3) * m3 * (dq1 + dq2/2) - a3/2 * (a1 * s23 + 2 * a2 * s3) * m3 * dq2;
C21 = a1/2 * (a2 * s2 * (m2 + 2 * m3) + a3 * s23 * m3) * (dq1 + dq2/2) + a1/4 * a3 * s23 * m3 * dq3;
C22 = -a1/4 * (a2 * m2 * s2 + 2 * a2 * m3 * s2 + a3 * m3 * s23) * dq1;
C23 = -a3 * m3 * ((a2 * s3 + a1/4 * s23) * dq1 + a2 * s3 * dq2 + a2/2 * s3 * dq3);
C31 = a3 * m3 * ((a1 * s23 + a2 * s3)/2 * dq1 + (a1 * s23 + 2 * a1 * s23)/4 * dq2 + ...
    (a1 * s23 + a3 * s3)/4 * dq3);
C32 = a3/2 * m3 * ((a2 * s3 - a1/2 * s23) * dq1 + a2 * s3 * dq2 + a2/2 * s3 * dq3);
C33 = -a3/4 * m3 * ((a1 * s23 + a2 * s3) * dq1 + a2 * s3 * dq2);

Dynamic_Mats.C = ...
    [C11 C12 C13;
     C21 C22 C23;
     C31 C32 C33];

% Gravity vector (G) computation
Dynamic_Mats.G = ...
    [m1/2 * a1 * c1 + m2/2 * (a1 * c1 + a2 * c12) + m3 * (a2 * c12 + a3/2 * c123);
     m2/2 * a2 * c12 + m3 * (a2 * c12 + a3/2 * c123);
     m3/2 * a3 * c123] * g;

% Perturbed Dynamic Matrices
% Parameters initialization with perturbations
a1 = SP.a(1) * SP.pert; a2 = SP.a(2) * SP.pert; a3 = SP.a(3) * SP.pert;
m1 = SP.m(1) * SP.pert; m2 = SP.m(2) * SP.pert; m3 = SP.m(3) * SP.pert;
g = SP.g * SP.pert;

% Perturbed Mass Matrix (Mh) computation
% (Similar to the Mass Matrix, but with perturbed parameters)
M11 = (m1/3 + m2 + m3) * a1^2 + (m2/3 + m3) * a2^2 + m3 * a3^2 + ...
    (m2 + 2 * m3) * a1 * a2 * c2 + a3 * m3 * (a1 * c23 + a2 * c3);
M12 = (m2/3 + m3) * a2^2 + m3 * a3 * (a3/3 + a2 * c23/2) + ...
    m2/2 * a1 * a2 * c2 + m3 * a2 * (a1 * c2 + a3 * c23);
M13 = m3 * a3 * (a3/3 + a2 * c23/2 + a2 * c3/2);
M22 = (m2/3 + m3) * a2^2 + m3 * a3^2/3 + m3 * a2 * a3 * c3;
M23 = m3 * a3 * (a3/3 + a2 * c3/2);
M33 = m3 * a3^2/3;

Dynamic_Mats.Mh = ...
    [M11 M12 M13;
     M12 M22 M23;
     M13 M23 M33];

% Perturbed Coriolis/centrifugal matrix (Ch) computation
% (Similar to the Coriolis/centrifugal matrix, but with perturbed parameters)
C11 = 0;
C12 = -a1 * (a2 * s2 * m2 + (2 * a2 * s2 + a3 * s23) * m3) * (dq1 + dq2/2) - a1/2 * a3 * s23 * m3 * dq3;
C13 = -a3 * (a1 * s23 + a2 + s3) * m3 * (dq1 + dq2/2) - a3/2 * (a1 * s23 + 2 * a2 * s3) * m3 * dq2;
C21 = a1/2 * (a2 * s2 * (m2 + 2 * m3) + a3 * s23 * m3) * (dq1 + dq2/2) + a1/4 * a3 * s23 * m3 * dq3;
C22 = -a1/4 * (a2 * m2 * s2 + 2 * a2 * m3 * s2 + a3 * m3 * s23) * dq1;
C23 = -a3 * m3 * ((a2 * s3 + a1/4 * s23) * dq1 + a2 * s3 * dq2 + a2/2 * s3 * dq3);
C31 = a3 * m3 * ((a1 * s23 + a2 * s3)/2 * dq1 + (a1 * s23 + 2 * a1 * s23)/4 * dq2 + ...
    (a1 * s23 + a3 * s3)/4 * dq3);
C32 = a3/2 * m3 * ((a2 * s3 - a1/2 * s23) * dq1 + a2 * s3 * dq2 + a2/2 * s3 * dq3);
C33 = -a3/4 * m3 * ((a1 * s23 + a2 * s3) * dq1 + a2 * s3 * dq2);

Dynamic_Mats.Ch = ...
    [C11 C12 C13;
     C21 C22 C23;
     C31 C32 C33];

% Perturbed gravity vector (Gh) computation
% (Similar to the gravity vector, but with perturbed parameters)
Dynamic_Mats.Gh = ...
    [m1/2 * a1 * c1 + m2/2 * (a1 * c1 + a2 * c12) + m3 * (a2 * c12 + a3/2 * c123);
     m2/2 * a2 * c12 + m3 * (a2 * c12 + a3/2 * c123);
     m3/2 * a3 * c123] * g;
end