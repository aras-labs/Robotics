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
x=-1:0.05:1; 
y=-1:0.05:1; 
z=[0 0 0 0];
si=[1 1 0 0
    1 0 1 0
    1 0 0 1];
theta=[ 0  30  30  30]*deg2rad;
for i=1:max(size(x));
  for j=1:max(size(y));
    for k=1:max(size(theta)); %phi(k) = 45*deg2rad; % for k=1:max(size(phi));
    Zi=[x(i);y(j);z(k)];
    Ri=sc2rot(si(:,k),theta(:,k));
    J=Jacobian(Zi,Ri);
 %   ME(i,j,k)=sqrt(max(eig(J'J)));
    MS(i,j,k)=max(svd(J));
    RC(i,j,k)=rcond(J'*J);
        end
  end
end

figure(1)
clf
    subplot(2,2,1)
    surfc(x,y,RC(:,:,1));
    colormap(hot)
    xlabel('x position ')
    ylabel('y position ')
    title(['\theta = 0 degrees'])
    zlabel(' 1/\kappa of J')

    subplot(2,2,2);
    surfc(x,y,RC(:,:,2));
    colormap(hot)
    xlabel('x position ')
    ylabel('y position ')
    title(['\theta_x= 30 degrees'])
    zlabel(' 1/\kappa of J')
  
    subplot(2,2,3);
    surfc(x,y,RC(:,:,3));
    colormap(hot)
    xlabel('x position ')
    ylabel('y position ')
    title(['\theta_y= 30 degrees'])
    zlabel(' 1/\kappa of J')

    subplot(2,2,4);
    surfc(x,y,RC(:,:,4));
    colormap(hot)
    xlabel('x position ')
    ylabel('y position ')
    title(['\theta_z= 30 degrees'])
    zlabel(' 1/\kappa of J')
figure(2)
clf
surfc(x,y,MS(:,:,2))
  colormap(hot)
    xlabel('x position ')
    ylabel('y position ')
    zlabel(' max singular value of J')
    title(['\theta_x = 30 degrees'])

%   This concludes the program
%