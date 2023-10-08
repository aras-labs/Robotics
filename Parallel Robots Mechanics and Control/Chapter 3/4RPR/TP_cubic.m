%% 
%   Parallel Robots: Mechanics and Control
%   Copyright Hamid D. Taghirad 2013
%
%   This program generates a cubic trajectory for the manipulator at time t
%

function [ixd,xd,dxd,d2xd]=TP_cubic(t)
%
%   Initialization
%

global Par
tpoints=Par.xd(:,1);
xpoints=Par.xd(:,2);
ypoints=Par.xd(:,3);
zpoints=Par.xd(:,4);


%
%   Finding the time section in which the trajectory must be generated
%
for i=1:max(size(tpoints));
    if t>= tpoints(i) && t<tpoints(i+1)
        t0=tpoints(i);tf=tpoints(i+1);
        x0=xpoints(i);xf=xpoints(i+1);
        y0=ypoints(i);yf=ypoints(i+1);
        z0=zpoints(i);zf=zpoints(i+1);
        break
    end
end

%
%   Generate the trajectory at the time with its derivatives
%

if t0==0,
a=[x0   y0  z0; 
    0   0   0;
    3/tf^2*(xf-x0)  3/tf^2*(yf-y0)  3/tf^2*(zf-z0);
    -2/tf^3*(xf-x0)  -2/tf^3*(yf-y0)  -2/tf^3*(zf-z0)];
else
A=[ 1   t0  t0^2    t0^3
    1   tf  tf^2    tf^3
    0   1   2*t0    3*t0^2
    0   1   2*tf    3*tf^2];
y=[ x0 y0 z0;
    xf yf zf;
    0 0 0;
    0 0 0];
a=A\y;                  % The parameters are found 
end

tix=    [t  t^2/2   t^3/3 t^4/4];
tx=     [1  t   t^2 t^3];
tdx=    [0  1   2*t 3*t^2];
td2x=   [0  0   2   6*t];

% Transposition is used to generate a column vector
ixd=    (tix*a)';           % The interal of the trajectory     
xd=     (tx*a)';            % The trajectory itself
dxd=    (tdx*a)';           % The derivative of trajectory
d2xd=   (td2x*a)';          % Second derivative of trajectory
