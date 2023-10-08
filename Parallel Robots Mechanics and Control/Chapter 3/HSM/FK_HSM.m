function th=FK_SGP(L,th_old);
%
%   Copyright Hamid D. Taghirad 2006
%
global Par,

lp=Par.lp;
lb=Par.lb;
ld=Par.ld;
lk=Par.lk;
alpha=Par.alpha;
k=Par.k;

%
%%
s2=(L(1)^2-L(2)^2-L(3)^2+L(4)^2)/(4*k(2));
c2=sqrt(1-s2^2);

th(1,2)=atan2(s2,c2);
%
%%

s3=(L(1)^2-L(2)^2-2*k(2)*s2)/(2*k(4)*c2);
c3=sqrt(1-s3^2);

th(1,3)=atan2(s3,c3);

%
%%
s1=(L(3)^2-L(1)^2+2*k(2)*s2)/(2*k(3)*c2);
c1=sqrt(1-s1^2);

th(1,1)=atan2(s1,c1);

end %end main function
