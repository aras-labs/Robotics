%% 
%   Parallel Robots: Mechanics and Control
%   Copyright Hamid D. Taghirad 2013
%
%   This program generates the singular points of 
%   the planar cable manipulator within its workspace.
%   Furthermore, it plots the sensitivity measure of 
%   the Jacobian for a typical trajectory.
%
%%
clear all
%   /initial values/
deg2rad=pi/180;
rad2deg=180/pi;
dt=0.001;
Tf=0.998;
N=Tf/dt;
% 
%   The coordinates of Ai's, Bi's, ai's and bi's at initial position
%   Consider the fixed coordinate is located at the center 
%   at initial position

X(:,1)=[0;0;0]; % pos/orientation of the first  moving platform center
RA = 1000;         % the Ai's circle radius 
RB = 10;           % the Bi's circle radius
Ath=[-135;-45;45;135]*deg2rad;
Bth=[-45;-135;135;45]*deg2rad;
for i=1:4;
     A(:,i)=[RA*cos(Ath(i)); RA*sin(Ath(i))];
 end
alpha_old = [30;150;-150;-30]*deg2rad; % found from first iteration
X_old=X;
%% Desired Trajectory 

%   The desired trajectory initial points
%     Time    x      y     \phi
SP.xd=[
        0     0      0        0*deg2rad
        1    100    -100     -45*deg2rad
        200   0      0        0*deg2rad
        300   0      0        0*deg2rad
        400   0      0        0*deg2rad
        500   0      0        0*deg2rad
        600   0      0        0*deg2rad
        10000 0      0        0*deg2rad
        ];
   
%%
%-------------------- Start iteration ------------------------ 
for i=1:N;
    t(i)=dt*(i);
%   The desired trajectory of the end effector xd=3xN vector 
%   And its derivative dxd=3xN vector
    wd=pi;
    deltax=-100;
    deltay=100;
    
    [Xd(:,i),dXd(:,i)]=TP_cubic_s(t(i),SP);
    Xd(:,i)=[Xd(1,i);Xd(2,i);X(3,1)+wd*t(i)-pi/2];    
    dXd(:,i)=[dXd(1,i);dXd(2,i);wd];
    xd=Xd;dxd=dXd;
%----------------------------------------------------------------
%   Solve the inverse kinematics
%
    [L(:,i),alpha(:,i)]=InvKin_4RPR(Xd(:,i),A,RB,Bth,alpha_old);
    B = Geometry_4RPR(Xd(:,i),RB,Bth);
    alpha_old=alpha(:,i);
%----------------------------------------------------------------    
%   Find the sensitivity measure of the Jacobian
%
     JMx=Jacobian(Xd(:,i),B,alpha(:,i));
     dL(:,i)=JMx*dXd(:,i);
     Sen(:,i)=rcond(JMx'*JMx);
end
%-------------------- End iteration ------------------------ 
%%
figure(1)
clf
subplot(211)
plot(t,Xd(1,:),t,Xd(2,:),'--',t,Xd(3,:)*rad2deg,'-.'),grid
title('Motion Trajectory')
legend('x_G','y_G','\phi')
xlabel('time(sec)')
ylabel('x_G, y_G (m) and \phi (deg)')
subplot(212)
semilogy(t,Sen), grid off
title('Sensitivity measure')
xlabel('time(sec)')
ylabel('1/cond(J)')

%%
%   3D Workspace and singularity analysis
%
%   Define a 3D workspace 
Xg=-50:10:50; 
Yg=-50:10:50;
phi=-pi:pi/4:pi;
clear Z 
NN=1;MM=1;KK=1;
alpha_old = [45;135;-135;-45]*deg2rad; % found from first iteration
for i=1:max(size(Xg));
  for j=1:max(size(Yg));
    for k=1:max(size(phi));
    Zi=[Xg(i);Yg(j);phi(k)];
    [Li, alphai]= InvKin_4RPR(Zi,A,RB,Bth,alpha_old);
    Bi = Geometry_4RPR(Zi,RB,Bth);
    alpha_old=alphai;
    JMi=Jacobian(Zi,Bi,alphai);
    if rcond(JMi'*JMi) < 1e-8 ;   % threshold of singularity
      Z(:,NN)=[Xg(i);Yg(j);phi(k)];
      NN=NN+1;
    end
   end 
 end
end
%%
figure(2)
clf
plot3(Z(1,:),Z(2,:),Z(3,:)*rad2deg,'o')
grid on
xlabel('Position x (m)')
ylabel('Position y (m)')
zlabel('Orientation \phi (degrees)')
title('Locations where singularity occurs')
%   This concludes the program
%