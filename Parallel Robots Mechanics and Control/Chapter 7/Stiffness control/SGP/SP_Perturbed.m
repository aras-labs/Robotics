%% 
%   Parallel Robots: Mechanics and Control
%   Copyright Hamid D. Taghirad 2013
%
%   This program perturbs the parameters generated in a structure.
%
function [SP] = SP_Perturbed(SP)

%% Attachment Points Coordinates

SP.RA  = SP.RA * SP.pert ;    % Perturbed Ai's circle radius (m)
SP.RB  = SP.RB * SP.pert ;    % Perturbed Bi's circle radius (m)SP.Ath = [0; 60; 120; 180; 240; 300] * deg2rad ; % Angle between Fixed Attachment Points and X axis of Fixed Coordinate System
for i = 1:6
  SP.A(:,i) = [SP.RA*cos(SP.Ath(i)); SP.RA*sin(SP.Ath(i)); 0] ;
  SP.B_B(:,i) = [SP.RB*cos(SP.Bth(i)); SP.RB*sin(SP.Bth(i)); 0] ;
end

%% Mass, Density and Inertia

SP.m_EF = SP.m_EF * SP.pert ;  % the Mass of the E.F. (Kg)
SP.I_EF = SP.I_EF * SP.pert ; % the Inertia Matrix of the E.F. relative to Moving Coordinate System (Kg.m^2)

for i = 1:SP.n
    SP.c1(:,i)   = SP.c1(:,i) * SP.pert;      % The center of Mass of ith Cylinder (m)
    SP.m1(:,i)   = SP.m1(:,i) * SP.pert;      % Mass of ith Cylinder (Kg)
    SP.I1xx(:,i) = SP.I1xx(:,i)* SP.pert ;    % Moment of inertia of ith symmetric Cylinder around moving x axis(Kg.m^2)
    SP.cp2(:,i)  = SP.cp2(:,i)* SP.pert;      % The center of Mass of ith Piston (m)
    SP.m2(:,i)   = SP.m2(:,i)* SP.pert;       % Mass of ith Piston (Kg)
    SP.I2xx(:,i) = SP.I2xx(:,i)* SP.pert;     % Moment of inertia of ith symmetric Piston around moving x axis (Kg.m^2)
    SP.Ixx(:,i)  = SP.I2xx(:,i) + SP.I1xx(:,i) ; 
            % Moment of inertia of ith symmetric Leg around moving x axis (Kg.m^2)
end

end