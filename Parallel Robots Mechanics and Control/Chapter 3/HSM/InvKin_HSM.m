function [L,D]=InvKin_SGP(theta);
%
%   Inverse Kinematics of the Hydraulic Shoulder Manipulator
%
%   Input arguments:
%   R   : The orientation matrix of the moving platform             2x4
%
%   Output argument:
%   L   : The lmb lengths    Direct computation         1x4
%   D   : The lmb lengths    Simplified computation     1x4
global Par,

lp=Par.lp;
lb=Par.lb;
ld=Par.ld;
lk=Par.lk;
alpha=Par.alpha;
k=Par.k;


s1=sin(theta(1));
c1=cos(theta(1));
s2=sin(theta(2));
c2=cos(theta(2));
s3=sin(theta(3));
c3=cos(theta(3));
sa=sin(alpha);
ca=cos(alpha);

P=lp*[s2;-c2*s1;c2*c1];
R=[c3*c2            -s3*c2          s2
    s3*c1+c3*s2*s1  c3*c1-s3*s2*s1  -c2*s1
    s3*s1-c3*s2*c1  c3*s1+s3*s2*c1  c2*c1];
A=lb*[sa -sa -sa sa
      -ca -ca ca ca
      0 0 0 0];
PB=[ 0 0 0 0
     -ld -ld ld ld
     lp-lk lp-lk lp-lk lp-lk];

for i= 1:4,
    LL=P+R*PB(:,i)-A(:,i);
    L(i)=sqrt(LL'*LL);
end
L=L';

D(1)=k(1)+k(2)*s2-k(3)*c2*s1+k(4)*s3*c2+k(5)*(c3*c1-s3*s2*s1);
D(2)=k(1)-k(2)*s2-k(3)*c2*s1-k(4)*s3*c2+k(5)*(c3*c1-s3*s2*s1);
D(3)=k(1)-k(2)*s2+k(3)*c2*s1+k(4)*s3*c2+k(5)*(c3*c1-s3*s2*s1);
D(4)=k(1)+k(2)*s2+k(3)*c2*s1-k(4)*s3*c2+k(5)*(c3*c1-s3*s2*s1);
D=sqrt(D)';
