%% 
%   Parallel Robots: Mechanics and Control
%   Copyright Hamid D. Taghirad 2013
%
%   This program generates the Jacobian matrix
%   of the non-redundant planar cable manipulator 
%
%%
function J = Jacobian(X,B,alpha);
%
%   Jacobian Matrix of the 4RPR parallel manipulator
%   function J = MJacobian(X,B,alpha);
%
%   Input arguments:
%
%   X    : The position/orientation of the Moving platform    3x1
%   B    : The Position of Bi's                               2x4   
%   alpha: The limb absolute angles                           4x1
%
%   Output argument:
%   J : The Jx-Jacobian matrix of non-redundantmanipulator    3x3

G=X(1:2,1);
for j=1:3,
     VE(:,j)=B(:,j)-G;
     VS(:,j)=[cos(alpha(j));sin(alpha(j))];     
end
E1=VE(:,1); E2=VE(:,2); E3=VE(:,3); 
S1=VS(:,1); S2=VS(:,2); S3=VS(:,3); 

ExS= [E1(1)*S1(2)-E1(2)*S1(1) 
      E2(1)*S2(2)-E2(2)*S2(1)
      E3(1)*S3(2)-E3(2)*S3(1)
     ];

 J=[VS(1,:)', VS(2,:)', ExS];

