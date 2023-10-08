%% 
%   Parallel Robots: Mechanics and Control
%   Copyright Hamid D. Taghirad 2013
%
%       This program generates a cubic trajectory for the manipulator.
%
function [xd,dxd,d2xd,ixd]=TP_cubic_s(t,Struct_Param)
%
%   Trajectory panning for positiion P=[x; y; z]'
%   and orientation represented by screw \btheta=\theta*[s_x; s_y; s_z]
%
tpoints=Struct_Param.xd(:,1);
xpoints=Struct_Param.xd(:,2);
ypoints=Struct_Param.xd(:,3);
zpoints=Struct_Param.xd(:,4);
sxpoints=Struct_Param.xd(:,5); 
sypoints=Struct_Param.xd(:,6);
szpoints=Struct_Param.xd(:,7);
thpoints=Struct_Param.xd(:,8);

for i=1:max(size(tpoints));
    if t>= tpoints(i) && t<=tpoints(i+1)
        t0=tpoints(i);tf=tpoints(i+1);
        x0=xpoints(i);xf=xpoints(i+1);
        y0=ypoints(i);yf=ypoints(i+1);
        z0=zpoints(i);zf=zpoints(i+1);
        sx0=sxpoints(i);sxf=sxpoints(i+1);
        sy0=sypoints(i);syf=sypoints(i+1);
        sz0=szpoints(i);szf=szpoints(i+1);
        th0=thpoints(i);thf=thpoints(i+1);
        break
    end
end
if t0==0,
a=[x0   y0  z0; 
    0   0   0;
    3/tf^2*(xf-x0)  3/tf^2*(yf-y0)  3/tf^2*(zf-z0);
    -2/tf^3*(xf-x0)  -2/tf^3*(yf-y0)  -2/tf^3*(zf-z0)];
s=[sx0   sy0  sz0; 
    0   0   0;
    3/tf^2*(sxf-sx0)  3/tf^2*(syf-sy0)  3/tf^2*(szf-sz0);
    -2/tf^3*(sxf-sx0)  -2/tf^3*(syf-sy0)  -2/tf^3*(szf-sz0)];

th=[th0;0;3/tf^2*(thf-th0);-2/tf^3*(thf-th0)];

else
A=[ 1   t0  t0^2    t0^3
    1   tf  tf^2    tf^3
    0   1   2*t0    3*t0^2
    0   1   2*tf    3*tf^2];
y=[ x0 y0 z0;
    xf yf zf;
    0 0 0;
    0 0 0];
z=[ sx0 sy0 sz0;
    sxf syf szf;
    0 0 0;
    0 0 0];
u=[ th0; thf; 0; 0];
a=A\y;
s=A\z;
th=A\u;
end

tix=    [t  t^2/2   t^3/3 t^4/4];
tx=     [1  t   t^2 t^3];
tdx=    [0  1   2*t 3*t^2];
td2x=   [0  0   2   6*t];
sd=     (tx*s)';

ixd=    [(tix*a)' ;(tix*th)'* sd];
xd=     [(tx*a)'  ;(tx*th)'*  sd];
dxd=    [(tdx*a)' ;(tdx*th)'* sd];
d2xd=   [(td2x*a)';(td2x*th)'*sd];




