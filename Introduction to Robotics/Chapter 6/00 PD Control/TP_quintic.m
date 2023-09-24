%% 
% 	An Introduction to Robotics, 
%   By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
%   Copyright ARAS @2023
%
%   This function generates a quintic trajectory 
%   with via points 

function [xd, dxd, d2xd] = TP_quintic(t, SP)
% TP_quintic generates a quintic polynomial trajectory for a given time t.
%
% Inputs:
%   - t: Current time.
%   - SP: Struct containing trajectory information.
%
% Outputs:
%   - xd: Position at time t.
%   - dxd: Velocity at time t.
%   - d2xd: Acceleration at time t.

% Extract trajectory points from the SP struct
tpoints = SP.xd(:, 1);  % Time points
xpoints = SP.xd(:, 2);  % X coordinates
ypoints = SP.xd(:, 3);  % Y coordinates
zpoints = SP.xd(:, 4);  % Z coordinates

% Find the segment of the trajectory that includes the current time t
for i = 1:max(size(tpoints))
    if t >= tpoints(i) && t <= tpoints(i+1)
        t0 = tpoints(i);
        tf = tpoints(i+1);
        x0 = xpoints(i);
        xf = xpoints(i+1);
        y0 = ypoints(i);
        yf = ypoints(i+1);
        z0 = zpoints(i);
        zf = zpoints(i+1);
        I = i;  % Current segment index
        I1 = i-1;  % Previous segment index
        break
    end
end

% Check if t0 is zero
if t0 == 0
    % Quintic polynomial coefficients when t0 is zero
    a = [x0   y0  z0; 
         0   0   0;
         0   0   0;
         10/tf^3*(xf-x0)     10/tf^3*(yf-y0)     10/tf^3*(zf-z0);
        -15/tf^4*(xf-x0)    -15/tf^4*(yf-y0)    -15/tf^4*(zf-z0);
         6/tf^5*(xf-x0)      6/tf^5*(yf-y0)      6/tf^5*(zf-z0)];
else
    % A matrix for non-zero initial time t0
    A = [1   t0  t0^2    t0^3    t0^4    t0^5
         1   tf  tf^2    tf^3    tf^4    tf^5
         0   1   2*t0    3*t0^2  4*t0^3  5*t0^4
         0   1   2*tf    3*tf^2  4*tf^3  5*tf^4
         0   0   2       6*t0    12*t0^2 20*t0^3    
         0   0   2       6*tf    12*tf^2 20*tf^3];
    
    % Construct a matrix for zero initial velocity and acceleration
    y = [x0 y0 z0;
         xf yf zf;
         0 0 0;
         0 0 0;
         0 0 0;
         0 0 0];
    
    % Calculate the polynomial coefficients using matrix division
    a = A \ y;
end

% Time-dependent coefficients for the quintic polynomial
tx =     [1  t       t^2     t^3     t^4     t^5];
tdx =    [0  1       2*t     3*t^2   4*t^3   5*t^4];
td2x =   [0  0       2       6*t     12*t^2  20*t^3];

% Calculate position, velocity, and acceleration at time t
xd =    (tx * a)';
dxd =   (tdx * a)';
d2xd =  (td2x * a)';

