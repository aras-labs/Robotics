%   Copyright Hamid D. Taghirad 2009
%
%   This program determoines the characteristics of 
%   the stiffness matrix of the SGP
clear all
%   /initial values/
global Par
deg2rad=pi/180;
rad2deg=180/pi;
% 
%   The coordinates of Ai's, Bi's,at initial position
%   Consider the fixed coordinate is located at the center 
%   at initial position
Par.RA=2;                   % the Ai's circle radius 
Par.RB=1;                   % the Bi's circle radius Par.Ath=[0; 60; 120; 180; 240; 300]*deg2rad; 
Par.Ath=[0; 60; 120; 180; 240; 300]*deg2rad;
Par.Bth=[30; 30; 150; 150; 270; 270]*deg2rad;
% Par.Ath=[300; 60; 60; 180; 180; 300]*deg2rad;
% Par.Bth=[0; 0; 120; 120; 240; 240]*deg2rad;
RA = Par.RA;          % the Ai's circle radius 
RB = Par.RB;           % the Bi's circle radius
Ath=Par.Ath;
Bth=Par.Bth;
for i=1:6;
  A(:,i)=[RA*cos(Ath(i)); RA*sin(Ath(i)); 0];   % notice no z
  B(:,i)=[RB*cos(Bth(i)); RB*sin(Bth(i)); 1];   % notice zb = 1 m
end
Par.A=A; Par.B=B;

%   3D Workspace and singularity analysis
%
%   Define a 3D workspace 
x=-1:0.1:1; 
y=-1:0.1:1; 
z=0.5:0.2:1.5; 
si=[1
    0 
    0];
theta=[60]*deg2rad; NN=1;
R=sc2rot(si,theta);

for i=1:max(size(x));
  for j=1:max(size(y));
    for k=1:max(size(z)); 
    Zi=[x(i);y(j);z(k)];
    J=Jacobian(Zi,R);
    if det(J) < 1e-17 ;
      Z(:,NN)=[x(i);y(j);z(k)];
      NN=NN+1;
    end
    end
  end
end

figure(1)
clf
plot3(Z(1,:),Z(2,:),Z(3,:),'o')
grid on
xlabel('Position x (m)')
ylabel('Position y (m)')
zlabel('Position z (m)')
title('Locations where singularity occurs')

%   This concludes the program
%