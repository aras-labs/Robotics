%% 
%   Parallel Robots: Mechanics and Control
%   Copyright Hamid D. Taghirad 2013
%
%   This program updates the perturbations in the parameters.
%
function [SP] = SP_update(SP, th)


%% Attachment Points Coordinates

% SP.RA  = SP.RA * SP.pert ;    % Perturbed Ai's circle radius (m)
% SP.RB  = SP.RB * SP.pert ;    % Perturbed Bi's circle radius (m)
% SP.Ath = SP.Ath ; % Angle between Fixed Attachment Points and X axis of Fixed Coordinate System
% SP.Bth = SP.Bth ; % Angle between Moving Attachment Points and x axis of Moving Coordinate System
% for i = 1:SP.n
%   SP.A(:,i) = [SP.RA*cos(SP.Ath(i)); SP.RA*sin(SP.Ath(i)); 0] ;
%   SP.B_B(:,i) = [SP.RB*cos(SP.Bth(i)); SP.RB*sin(SP.Bth(i)); 0] ;
% end

%% Mass, Density and Inertia

SP.m   = th(1) ;    % Updated Mass of the E.F. (Kg) by adaptation law
SP.Im  = th(2) ;    % Updated Inertia Matrix by adaptation law
SP.rho = th(3) ;    % Updated cable density (Kg/m) by adaptation law

end