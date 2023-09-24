%% 
% 	An Introduction to Robotics, 
%   By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
%   Copyright ARAS @2023
%
%   This program generates a quintic trajectory 
%   with zero initial velocity and acceleration 

function [xd,dxd,d2xd]=TP_quintic(t,SP)
% TP_quintic computes a quintic trajectory for a point in 3D space.
%
%   [xd,dxd,d2xd] = TP_quintic(t, SP)
%
%   Inputs:
%   - t: Current time.
%   - SP: Trajectory planning parameters.
%
%   Outputs:
%   - xd: The position at time t.
%   - dxd: The velocity at time t.
%   - d2xd: The acceleration at time t.

% Extract trajectory points and corresponding time intervals
tpoints = SP.xd(:,1);
xpoints = SP.xd(:,2);
ypoints = SP.xd(:,3);
zpoints = SP.xd(:,4);

% Determine the time interval [t0, tf] that t belongs to
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
        break
    end
end

if t0 == 0
    % Quintic polynomial with zero initial velocity and acceleration
    a = [x0   y0  z0; 
         0    0   0;
         0    0   0;
         10/tf^3*(xf-x0) 10/tf^3*(yf-y0) 10/tf^3*(zf-z0);
         -15/tf^4*(xf-x0) -15/tf^4*(yf-y0) -15/tf^4*(zf-z0);
         6/tf^5*(xf-x0)   6/tf^5*(yf-y0)   6/tf^5*(zf-z0)];
else
    % Quintic polynomial with specified initial and final conditions
    A = [1   t0  t0^2  t0^3  t0^4  t0^5
         1   tf  tf^2  tf^3  tf^4  tf^5
         0   1   2*t0  3*t0^2 4*t0^3 5*t0^4
         0   1   2*tf  3*tf^2 4*tf^3 5*tf^4
         0   0   2     6*t0   12*t0^2 20*t0^3    
         0   0   2     6*tf   12*tf^2 20*tf^3];
    y = [x0 y0 z0;
         xf yf zf;
         0 0 0;
         0 0 0;
         0 0 0;
         0 0 0];
    a = A\y;
end

% Calculate position, velocity, and acceleration at time t
tx = [1 t t^2 t^3 t^4 t^5];
tdx = [0 1 2*t 3*t^2 4*t^3 5*t^4];
td2x = [0 0 2 6*t 12*t^2 20*t^3];

xd = (tx*a)';
dxd = (tdx*a)';
d2xd = (td2x*a)';