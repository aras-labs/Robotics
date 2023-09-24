%%
% 	An Introduction to Robotics,
%   By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
%   Copyright ARAS @2023
%
%   This code improves the representation of symbolic matrices by numerically
%   substituting the symbolic expressions with their approximate decimal values
%   using the Variable Precision Arithmetic (VPA) toolbox.
%
%   Inputs:
%   - T3: The symbolic matrix to be numerically substituted
%   - n: The number of rows in the matrix
%   - m: The number of columns in the matrix
%
%   Output:
%   - T_real: The matrix with symbolic expressions numerically substituted.
%
%   Note: Symbolic matrices in MATLAB may contain exact symbolic expressions,
%   which can be challenging to work with in numerical computations. This code
%   replaces the symbolic expressions with their approximate decimal values using
%   the VPA toolbox, resulting in a matrix with numerical values that are easier
%   to analyze and perform calculations with.

function [T_real] = Matrix_Vpa_Numeric(T3, n, m)
% Numerically substitute the symbolic expressions in the matrix.
for i=1:n
    % Loop over the rows of the matrix.
    for j=1:m
        % Loop over the columns of the matrix.
        C = T3(i,j);
        % Store the symbolic expression in C.
        c=vpa(C,2);
        % Convert the symbolic expression to its approximate decimal value using VPA.
        if abs(c) < 0.001
            % If the absolute value of c is less than 0.001, set c to 0.
            c=0;
        end
        % Store the numerically substituted value in T3_real.
        T3_real(i,j)=c;  %#ok
    end
end

% Convert the matrix with numerically substituted values (T3_real) to its
% approximate decimal representation using VPA.
T_real=vpa(T3_real,3);
end