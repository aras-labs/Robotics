%% 
%   Robotics Course, Professor: Prof. Hamid D. Taghirad
%   Aras.kntu.ac.ir/education/robotics
%   Copyright Ali.Hassani 2020
%
%   This code provides a better representation of the vector symbolic formulas in MATLAB
%%
function [P_real] = Vector_Vpa(P,n)

for i=1:n
        [C,T] = coeffs(P(i));
        c=vpa(C,2);
        if abs(c) < 0.001
            c=0;
        end
            P3_real(i,1)=dot(c,T);

    end


P_real=vpa(P3_real,3);
end


