function J = Jacobian(theta,L,s);
%
%   Jacobian Matrix of the shoulder manipulator
%   function J = Jacobian(theta,L,s);
%
%   Input arguments:
%
%   theta: The orientation angle theta1, theta2, theta 3            3x1
%   L    : The Limb length                                          4x1
%   s    : The limb direction unit vectos                           3x4
%
%   Output argument:
%   J : The Jx- Jacobian matrix of shoulder manipulator             4x3
global Par,

lp=Par.lp;
lb=Par.lb;
ld=Par.ld;
lk=Par.lk;
alpha=Par.alpha;


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
PA=R*PB;

Jv=[s(:,1)' PA(:,1)'
    s(:,2)' PA(:,2)'
    s(:,3)' PA(:,3)'
    s(:,4)' PA(:,4)'
];
Jp=lp*[0 c2*c1 c2*s1
      -c2*c1 0 s2
      -c2*s1 s2 0];
Jp=[Jp;eye(3,3)];

J=Jv*Jp;
