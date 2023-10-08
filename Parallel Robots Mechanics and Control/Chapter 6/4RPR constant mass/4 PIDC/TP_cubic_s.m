%% 
%   Parallel Robots: Mechanics and Control
%   Copyright Hamid D. Taghirad 2013
%
%       This program generates a cubic trajectory for the manipulator.
%
function [xd,dxd,d2xd,ixd]=TP_cubic_s(t,SP)
%
%   Trajectory panning for positiion P=[x; y; z]'
%   and orientation represented by screw \btheta=\theta*[s_x; s_y; s_z]
%
tpoints=SP.xd(:,1);
xpoints=SP.xd(:,2);
ypoints=SP.xd(:,3);
zpoints=SP.xd(:,4);

for i=1:max(size(tpoints));
    if t>= tpoints(i) && t<=tpoints(i+1)
        t0=tpoints(i);tf=tpoints(i+1);
        x0=xpoints(i);xf=xpoints(i+1);
        y0=ypoints(i);yf=ypoints(i+1);
        z0=zpoints(i);zf=zpoints(i+1);
        break
    end
end
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
a=A\y;
end

tix=    [t  t^2/2   t^3/3 t^4/4];
tx=     [1  t   t^2 t^3];
tdx=    [0  1   2*t 3*t^2];
td2x=   [0  0   2   6*t];

ixd=   (tix*a)';
xd=    (tx*a)';
dxd=   (tdx*a)';
d2xd=  (td2x*a)';




