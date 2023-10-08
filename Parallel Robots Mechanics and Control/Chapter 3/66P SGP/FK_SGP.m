function X=FK_SGP(D,x_old);
%
%   Copyright Hamid D. Taghirad 2006
%
global Par,

A=Par.A;
B=Par.B;
mu=Par.mu;
sth_old=x_old(4:7);
p_old=x_old(1:3);

for i=1:6
   Q(i,1)=1;
   Q(i,2)=2*A(1,i);
   Q(i,3)=2*A(2,i);
   Q(i,4)=-2*mu*A(1,i)^2;
   Q(i,5)=-2*mu*A(1,i)*A(2,i);
   Q(i,6)=-2*mu*A(2,i)^2;
   d(i,1)=D(i)^2-(1+mu^2)*(A(1,i)^2+A(2,i)^2); 
end
W =inv(Q)*d;
BB=(W(4)+W(6));
CC=W(4)*W(6)-(W(5)^2)/4;

cth=(BB-sqrt(BB^2-4*CC))/2;
th=acos(cth);
vth=1-cth;

if abs(W(4)-cth) > 1e-4,
    sx=sqrt((W(4)-cth)/vth);
    sy=W(5)/(2*sx*vth);
    sz=sqrt(abs(1-sx^2-sy^2));
elseif abs(W(6)-cth) > 1e-4,
    sy=sqrt((W(6)-cth)/vth);
    sx=W(5)/(2*sy*vth);
    sz=sqrt(abs(1-sx^2-sy^2));
else
    sx=0;sy=0;sz=1;
end


s=[sx sx -sx -sx
   sy sy -sy -sy
   sz -sz sz -sz];
sth=[s;[th th th th]];

for i=1:4,
    if norm(sth(:,i) - sth_old) < 1e-0,
       R=sc2rot(s(:,i),th);
       sth_out=sth(:,i);
    end
end

u=[1 0 0]*(mu*R'-eye(3,3)); u=u';
v=[0 1 0]*(mu*R'-eye(3,3)); v=v';
den=(u'*u)*(v'*v)-(u'*v)^2;
r0=((v'*v)*W(2)-(u'*v)*W(3))*u;
r0=r0+(-(u'*v)*W(2)+(u'*u)*W(3))*v;
r0=r0/den;
r1=cross(u,v);
r1=r1/norm(r1);
tt=sqrt(W(1)-r0'*r0);
Point(:,1)=r0-tt*r1;
Point(:,2)=r0+tt*r1;
for i=1:2;
    if norm(Point(:,i)-p_old) < 1e-1
        P_out=Point(:,i);
    end
end
X=[P_out;sth_out];

end %end main function
