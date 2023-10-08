%%
%   Parallel Robots: Mechanics and Control
%	Copyright Hamid D. Taghirad 2013
%
%   This program verifies the singularity free workspace of 
%   the shoulder manipulator

%%
clear all
global Par
%   /initial values/
deg2rad=pi/180;
rad2deg=180/pi;
Par.lp=0.25;
Par.lb=0.2;
Par.ld=0.05;
Par.lk=0.01;
Par.alpha=30*deg2rad;

%%
%   Define a 3D workspace for the shoulder manipulator
%   with grid points
%
th1=-pi/3:pi/60:pi/3; th1=th1';
th=[th1 th1 th1]; N=max(size(th1)); NN=1;
for i=1:N;
  for j=1:N;
    for k=1:N;
        theta=[th1(i);th1(j);th1(k)];
    [Li, si]= InvKin_HSM(theta);
    Ji=Jacobian_HSM(theta,Li,si);
     if rcond(Ji'*Ji) < 1e-8 ;      % Singularity threshold
        Z(:,NN)=theta;
        NN=NN+1;
     end
    end
   end
  end

%%
if NN==1;
    disp('There is no singularity configuration within the workspace')
else
    Z
end

% This concludes this program


%   This concludes the program
%