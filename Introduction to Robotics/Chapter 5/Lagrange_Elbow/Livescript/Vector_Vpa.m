%% 
%   Robotics Course, Professor: Prof. Hamid D. Taghirad
%   Aras.kntu.ac.ir/education/robotics
%   Copyright Ali.Hassani 2020
%
%  This code provides a better representation of the vector symbolic formulas in MATLAB
% Vector_Vpa - Evaluate a symbolic vector using VPA precision
%
% Syntax:
%   P_real = Vector_Vpa(P, n)
%
% Inputs:
%   P - Symbolic vector of expressions
%   n - Number of elements in the vector
%
% Outputs:
%   P_real - Numerical vector evaluated with specified precision
%
% Description:
%   This function evaluates a symbolic vector using VPA (Variable Precision
%   Arithmetic) precision and returns the numerical result with the specified
%   precision.
%
%   Author: Ali Hassani
%   Copyright 2020, Aras.kntu.ac.ir/education/robotics
%

function [P_real] = Vector_Vpa(P, n)
    P3_real = zeros(n, 1);  % Initialize the result vector
    
    for i = 1:n
        [C, T] = coeffs(P(i));
        c = vpa(C, 2);  % Use 2 digits of precision
        
        % If the absolute value of c is less than 0.001, set it to 0
        if abs(c) < 0.001
            c = 0;
        end
        
        % Calculate the dot product of c and T
        P3_real(i, 1) = dot(c, T);
    end

    % Evaluate the result vector with 3 digits of precision
    P_real = vpa(P3_real, 3);
end
