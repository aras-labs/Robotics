function J = Jacobian(X,R);
%
%   Jacobian Matrix of the Stewart Gough Platform
%   function J = Jacobian(X,B);
%
%   Input arguments:
%
%   X    : The position/orientation of the Moving platform    3x1
%   R    : The orientation matrix of the moving platform      3x3   
%
%   Output argument:
%   J : The J-Jacobian matrix of SGP         6x6
%
global Par,
A=Par.A;
Bfix=Par.B;
for j=1:6,
    B(:,j)=X+R*Bfix(:,j);
    VE(:,j)=B(:,j)-X;
    VS(:,j)=(B(:,j)-A(:,j))./norm(B(:,j)-A(:,j));
    VT(:,j)=cross(VE(:,j),VS(:,j));
end
 J=[VS',VT'];

