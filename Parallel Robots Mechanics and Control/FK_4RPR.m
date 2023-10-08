%% 
%   Parallel Robots: Mechanics and Control
%   Copyright Hamid D. Taghirad 2013
%
%   This program solves the forward kinematics 
%   of the planar cable manipulator 
%
%%
%   Input arguments:
%   L  : The limbs length                                   1x4
%   A  :  The position of Ai's                              2x3
%   RB  : The circle radius of Bi's                         1x1
%   Bth : The configuration angles of Bi's                  1x3
%   x_old: The previous X to wrap the phi                   1x4
%
%   Output argument:
%   X = [Gx;Gy;phi]                                         3x1
%   Gx:The x position of the center of the moving platform  1x1 
%   Gy:The y position of the center of the moving platform  1x1 
%   phi: The orientation of the moving platform             1x1
%
%%
function X=FK_3RPR(L,A,RB,Bth,X_old);

x=X_old(3);
X(3)=fzero(@FKfun, x);
    function out=FKfun(x)  
        phi=x;
        for i=1:4;          
            Phi(i)=Bth(i) + phi;
            xx(i)=A(1,i)-RB*cos(Phi(i));
            yy(i)=A(2,i)-RB*sin(Phi(i));
            r(i) = -2*xx(i);
            s(i) = -2*yy(i);
            u(i) = xx(i)^2 + yy(i)^2 - L(i)^2;
        end
        R=[r(1)-r(2) s(1)-s(2)
           r(2)-r(3) s(2)-s(3)
           r(3)-r(4) s(3)-s(4)];
          U=[u(2)-u(1)
             u(3)-u(2)
             u(4)-u(3)];
        v=pinv(R)*U;

        for i=1:4;
        g(i) = v(1)^2+v(2)^2+r(i)*v(1)+s(i)*v(2)+u(i);
        end
        %out=sum(g);
        out=g(1);
    end % End nested function
    %Calculates the X and Y coordinates
  X(1:2)=v;
end %end main function
