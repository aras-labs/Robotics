%% 
%   Parallel Robots: Mechanics and Control
%   Copyright Hamid D. Taghirad 2013
%
%   This program executes the inverse kinematics 
%   of the planar cable manipulator 
%
%%
%   Input arguments:
%   X   : The position and orientation vector of the moving 
%         platform                                          1x3
%   A   : The position of the base coordinates:             2x4
%   RB  : The circle radius of B'i's                        1x1
%   Bth : The configuration angles of Bi                    4x1
%   alpha_old: The previous alpha to wrap the alpha         1x4

%   Output argument:
%   L     : The lmb lengths                                 1x4
%   alpha : The angle of the limbs                          1x4

function [L,alpha]=InvKin_4RPR(X,A,RB,Bth,alpha_old);

threshold = pi/6;   % threshold to wrap alpha
xg=X(1);
yg=X(2);
phi=X(3);

for i= 1:4,
    Phi(i)=Bth(i)+ phi;
    x(i) = xg - A(1,i) + RB*cos(Phi(i));
    y(i) = yg - A(2,i) + RB*sin(Phi(i));
    L(i)=sqrt(x(i)^2+y(i)^2);
    alpha(i)=atan2(y(i),x(i));

    if abs(alpha(i) - alpha_old(i)) > threshold;
        if alpha(i) < alpha_old
            alpha(i)=alpha(i) + 2*pi;
        else
            alpha(i)=alpha(i) - 2*pi;
        end
    end
end
L=L';alpha=alpha';