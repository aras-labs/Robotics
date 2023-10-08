%% 
%   Parallel Robots: Mechanics and Control
%   Copyright Hamid D. Taghirad 2013
%
%   This program plots the reciprocal of the condition number of 
%   the planar cable manipulator within its workspace for four different 
%   constant orientations. Furthermore, it compares the stiffness values
%   for redundant and non-redundant structures.
%
%%
%   /initial values/
deg2rad=pi/180;
rad2deg=180/pi;
%
%   The coordinates of Ai's, Bi's, ai's and bi's at initial position
%   Consider the fixed coordinate is located at the center 
%   at initial position
%
X(:,1)=[0;0;0]; % pos/orientation of the first  moving platform center
x(:,1)=[0;0;0]; % pos/orientation of the second moving platform center
RA = 900;         % the Ai's circle radius 
RB = 10;           % the Bi's circle radius
Ath=[-135;-45;45;135]*deg2rad;
Bth=[-45;-135;135;45]*deg2rad;
 for i=1:4;
     A(:,i)=[RA*cos(Ath(i)); RA*sin(Ath(i))];
 end
alpha_old = [30;150;-150;-30]*deg2rad; % found from first iteration
beta_old  = alpha_old;

%%
%   3D Workspace and singularity analysis
%
%   Define a 3D workspace 
Xg=-800:50:800; 
Yg=-800:50:800;
phi=[0 45 60 75]*deg2rad;
alpha_old = [45;135;-135;-45]*deg2rad; % found from first iteration
beta_old=alpha_old;
for i=1:max(size(Xg));
  for j=1:max(size(Yg));
    for k=1:max(size(phi)); %phi(k) = 45*deg2rad; % for k=1:max(size(phi));
    Zi=[Xg(i);Yg(j);phi(k)];
    [Li, alphai]= InvKin_4RPR(Zi,A,RB,Bth,alpha_old);
    Bi = Geometry_4RPR(Zi,RB,Bth);
    alpha_old=alphai;
    J=Jacobian(Zi,Bi,alphai);       % redundant structure
    J3=Jacobian_3(Zi,Bi,alphai);    % nonredundant structure
    RC(i,j,k)=rcond(J'*J);
    RC3(i,j,k)=rcond(J3'*J3);
    end
  end
end
%%
figure(1)
clf
    subplot(2,2,1);
    surfc(Xg,Yg,RC(:,:,1));
    colormap(hot)
    xlabel('x position ')
    ylabel('y position ')
    title(['\phi= ',num2str(phi(1)*rad2deg),' (deg)'])
    zlabel(' 1/\kappa of J')
    subplot(2,2,2);
    surfc(Xg,Yg,RC(:,:,2));
    colormap(hot)
    xlabel('x position ')
    ylabel('y position ')
    title(['\phi= ',num2str(phi(2)*rad2deg),' (deg)'])
    zlabel(' 1/\kappa of J')
    subplot(2,2,3);
    surfc(Xg,Yg,RC(:,:,3));
    colormap(hot)
    xlabel('x position ')
    ylabel('y position ')
    title(['\phi= ',num2str(phi(3)*rad2deg),' (deg)'])
    zlabel(' 1/\kappa of J')
    subplot(2,2,4);
    surfc(Xg,Yg,RC(:,:,4));
    colormap(hot)
    xlabel('x position ')
    ylabel('y position ')
    title(['\phi= ',num2str(phi(4)*rad2deg),' (deg)'])
    zlabel(' 1/\kappa of J')
    
%%
figure(2)
clf
surf(Xg,Yg,RC(:,:,1))
hold
surf(Xg,Yg,RC3(:,:,1))
  colormap(hot)
    xlabel('x position ')
    ylabel('y position ')
    zlabel(['\phi= ',num2str(phi(1)*rad2deg),' (deg)'])
    title(' 1/\kappa of J for redundant anf non-redundant manipulator')

%   This concludes the program
