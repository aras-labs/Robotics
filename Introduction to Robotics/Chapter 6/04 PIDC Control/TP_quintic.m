%% 
% 	An Introduction to Robotics, 
%   By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
%   Copyright ARAS @2023
%
%   This program generates a quintic trajectory 
%   with zero initial velocity and acceleration 

% function [xd,dxd,d2xd]=TP_quintic(t,SP)
% %
% %   Trajectory planning for joints qd 
% %
% tpoints=SP.xd(:,1);
% xpoints=SP.xd(:,2);
% ypoints=SP.xd(:,3);
% zpoints=SP.xd(:,4);
% 
% for i=1:max(size(tpoints));
%     if t>= tpoints(i) && t<=tpoints(i+1)
%         t0=tpoints(i);tf=tpoints(i+1);
%         x0=xpoints(i);xf=xpoints(i+1);
%         y0=ypoints(i);yf=ypoints(i+1);
%         z0=zpoints(i);zf=zpoints(i+1);
% %        I=i;I1=i-1;T0=tpoints(i)
%         break
%     end
% end
% if t0==0,
% a=[x0   y0  z0; 
%     0   0   0;
%     0   0   0;
%     10/tf^3*(xf-x0)     10/tf^3*(yf-y0)     10/tf^3*(zf-z0);
%     -15/tf^4*(xf-x0)   -15/tf^4*(yf-y0)    -15/tf^4*(zf-z0);
%     6/tf^5*(xf-x0)      6/tf^5*(yf-y0)      6/tf^5*(zf-z0)];
% else
% A=[ 1   t0  t0^2    t0^3    t0^4    t0^5
%     1   tf  tf^2    tf^3    tf^4    tf^5
%     0   1   2*t0    3*t0^2  4*t0^3  5*t0^4
%     0   1   2*tf    3*tf^2  4*tf^3  5*tf^4
%     0   0   2       6*t0    12*t0^2 20*t0^3    
%     0   0   2       6*tf    12*tf^2 20*tf^3];
% %
% %   Zero initial velocity and acceleration
% %
% y=[ x0 y0 z0;
%     xf yf zf;
%     0 0 0;
%     0 0 0;
%     0 0 0;
%     0 0 0];
% a=A\y;
% end
% 
% %tix=    [t  t^2/2   t^3/3   t^4/4   t^5/5   t^6/6];
% %ti0=    [T0  T0^2/2   T0^3/3   T0^4/4   T0^5/5   T0^6/6];
% tx=     [1  t       t^2     t^3     t^4     t^5];
% tdx=    [0  1       2*t     3*t^2   4*t^3   5*t^4];
% td2x=   [0  0       2       6*t     12*t^2  20*t^3];
% 
% xd=    (tx*a)';
% dxd=   (tdx*a)';
% d2xd=  (td2x*a)';
% %ixd=   (tix*a)'-(ti0*a)';

function [xd, dxd, d2xd] = TP_quintic(t, SP)
% TP_quintic generates a quintic polynomial trajectory for joint positions.
%
%   [xd, dxd, d2xd] = TP_quintic(t, SP)
%
%   Inputs:
%   - t: Current time.
%   - SP: Struct containing trajectory points and parameters.
%
%   Outputs:
%   - xd: Joint position at time t.
%   - dxd: First derivative (velocity) of joint position at time t.
%   - d2xd: Second derivative (acceleration) of joint position at time t.

% Extract trajectory points and parameters
tpoints = SP.xd(:, 1); % Time points
xpoints = SP.xd(:, 2); % X-coordinate trajectory points
ypoints = SP.xd(:, 3); % Y-coordinate trajectory points
zpoints = SP.xd(:, 4); % Z-coordinate trajectory points

% Determine the segment of the trajectory for the current time
for i = 1:max(size(tpoints))
    if t >= tpoints(i) && t <= tpoints(i+1)
        t0 = tpoints(i); % Initial time of the segment
        tf = tpoints(i+1); % Final time of the segment
        x0 = xpoints(i); % Initial X-coordinate
        xf = xpoints(i+1); % Final X-coordinate
        y0 = ypoints(i); % Initial Y-coordinate
        yf = ypoints(i+1); % Final Y-coordinate
        z0 = zpoints(i); % Initial Z-coordinate
        zf = zpoints(i+1); % Final Z-coordinate
        break
    end
end

% Compute quintic polynomial coefficients for trajectory generation
if t0 == 0
    % If t0 is zero, ensure zero initial velocity and acceleration
    a = [x0, y0, z0; 
         0, 0, 0;
         0, 0, 0;
         10/tf^3 * (xf - x0), 10/tf^3 * (yf - y0), 10/tf^3 * (zf - z0);
        -15/tf^4 * (xf - x0), -15/tf^4 * (yf - y0), -15/tf^4 * (zf - z0);
         6/tf^5 * (xf - x0), 6/tf^5 * (yf - y0), 6/tf^5 * (zf - z0)];
else
    % If t0 is non-zero, compute coefficients based on initial and final conditions
    A = [1, t0, t0^2, t0^3, t0^4, t0^5;
         1, tf, tf^2, tf^3, tf^4, tf^5;
         0, 1, 2*t0, 3*t0^2, 4*t0^3, 5*t0^4;
         0, 1, 2*tf, 3*tf^2, 4*tf^3, 5*tf^4;
         0, 0, 2, 6*t0, 12*t0^2, 20*t0^3;
         0, 0, 2, 6*tf, 12*tf^2, 20*tf^3];
    
    % Set initial conditions (position and zero initial velocity/acceleration)
    y = [x0, y0, z0;
         xf, yf, zf;
         0, 0, 0;
         0, 0, 0;
         0, 0, 0;
         0, 0, 0];
    
    % Solve for quintic polynomial coefficients
    a = A \ y;
end

% Evaluate quintic polynomial for joint position, velocity, and acceleration
tx = [1, t, t^2, t^3, t^4, t^5];
tdx = [0, 1, 2*t, 3*t^2, 4*t^3, 5*t^4];
td2x = [0, 0, 2, 6*t, 12*t^2, 20*t^3];

% Compute joint position (xd), velocity (dxd), and acceleration (d2xd)
xd = (tx * a)';
dxd = (tdx * a)';
d2xd = (td2x * a)';

