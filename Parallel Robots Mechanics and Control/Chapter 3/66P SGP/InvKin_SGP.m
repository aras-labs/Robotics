function L=InvKin_SGP(x,R);
%
%   Inverse Kinematics of the SGP
%
%   Input arguments:
%   x   : The position vector of the moving platform                                          1x3
%   R   : The orientation matrix of the moving platform             2x4
%
%   Output argument:
%   L     : The lmb lengths                                 1x4

global Par,

A=Par.A;
B=Par.B;
BA=R*B;
for i= 1:6,
    L(i)=sqrt(x'*x+B(:,i)'*B(:,i) + A(:,i)'*A(:,i) ...
        + 2*x'*BA(:,i) -2*x'*A(:,i)-2*BA(:,i)'*A(:,i));
end
L=L';