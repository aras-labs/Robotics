%% 
%   Parallel Robots: Mechanics and Control
%   Copyright Hamid D. Taghirad 2013
%
%       This program generates a cubic trajectory for the Euler Angles
%
function [xd,dxd,d2xd,ixd]=TP_cubic_euler(t,SP)
%
%   Trajectory panning for positiion P=[x; y; z]'
%   and orientation represented by euler angle 
%
tpoints=SP.xd(:,1);
xpoints=SP.xd(:,2);
ypoints=SP.xd(:,3);
zpoints=SP.xd(:,4);

thpoints=SP.xd(:,8);

[n gb]=size(SP.xd);
% generate the screw axes
bs=[SP.xd(:,5) SP.xd(:,6) SP.xd(:,7)];  

% make sure it is normalized and not zero  
for i=1:n;

nbs(i)=norm(bs(i,:));
if nbs(i) ==0,
    bs(i,:)=[1 0 0];
else
    bs(i,:) =bs(i,:) /nbs(i);
end
end
sxpoints=bs(:,1); 
sypoints=bs(:,2); 
szpoints=bs(:,3); 

for i=1:max(size(tpoints))-1;
    if t>= tpoints(i) && t<=tpoints(i+1)
        t0=tpoints(i);tf=tpoints(i+1);
        x0=xpoints(i);xf=xpoints(i+1);
        y0=ypoints(i);yf=ypoints(i+1);
        z0=zpoints(i);zf=zpoints(i+1);
        th0=thpoints(i);thf=thpoints(i+1);
        Sx0 = sxpoints(i);Sxf=sxpoints(i+1);
        Sy0 = sypoints(i);Syf=sypoints(i+1);
        Sz0 = szpoints(i);Szf=szpoints(i+1);
        R0   = sc2rot([Sx0;Sy0;Sz0],th0);
        Rf   = sc2rot([Sxf;Syf;Szf],thf);
        phi0 = rot2euler(R0);
        phif = rot2euler(Rf);
        sx0 = phi0(1); sy0 = phi0(2); sz0 = phi0(3); 
        sxf = phif(1); syf = phif(2); szf = phif(3); 
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
a=A\y;
s=A\z;
end

tix=    [t  t^2/2   t^3/3   t^4/4];
tx=     [1  t       t^2     t^3  ];
tdx=    [0  1       2*t     3*t^2];
td2x=   [0  0       2       6*t  ];

ixd=    [(tix*a)' ;(tix*s)' ];
xd=     [(tx*a)'  ;(tx*s)'  ];
dxd=    [(tdx*a)' ;(tdx*s)' ];
d2xd=   [(td2x*a)';(td2x*s)'];




