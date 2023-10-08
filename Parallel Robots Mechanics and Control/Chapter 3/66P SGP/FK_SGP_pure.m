function X=FK_SGP(D,x_old);
%
%   Copyright Hamid D. Taghirad 2006
%
global Par,
x=x_old;
A=Par.A;
B=Par.B;

X=fsolve(@FKfun, x);
    function F=FKfun(x)  
        p=x(1:3);
        s=x(4:6);
%        shat=s./norm(s); % to make sure s is normalized
        th=x(7);
        R=sc2rot(s,th);
        BA=R*B;
        for i=1:6,
            f(:,i)=p+BA(:,i)-A(:,i);
            F(i)=-D(i)^2 +f(:,i)'*f(:,i);
        end
        F(7) = s'*s-1;
    end % End nested function
    % Other Calculations
    %
end %end main function
