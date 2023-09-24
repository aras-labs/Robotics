%% 
% 	An Introduction to Robotics, 
%   By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
%   Copyright ARAS @2023
%
%   This code improves the representation of symbolic matrices in MATLAB
%   by converting them to their approximate decimal representation using 
%   the Variable Precision Arithmetic (VPA) toolbox.
%
%   Inputs:
%   - T3: The symbolic matrix to be converted
%   - n: The number of rows in the matrix
%   - m: The number of columns in the matrix
%
%   Output:
%   - T_real: The approximate decimal representation of the symbolic matrix.
%
%   Note: Symbolic matrices in MATLAB may contain exact symbolic expressions,
%   which can be challenging to work with in numerical computations. This code
%   converts the symbolic matrix to its decimal approximation using the VPA
%   toolbox, making it easier to analyze and perform calculations with.

function [T_real] = Matrix_Vpa(T3, n, m)
    % Convert the symbolic matrix to its approximate decimal representation.
    for i = 1:n
        % Loop over the rows of the matrix.
        for j = 1:m
            % Loop over the columns of the matrix.
            [C, T] = coeffs(T3(i,j));
            % Extract the coefficients and the symbolic terms of T3(i,j).
            c = vpa(C, 2);
            % Convert the coefficients to their approximate decimal values using VPA.
            if abs(c) < 0.001
                % If the absolute value of c is less than 0.001, set c to 0.
                c = 0;
            end
            % Calculate the dot product of c and T, and store it in T3_real(i,j).
            T3_real(i,j) = dot(c, T);  %#ok
        end
    end
    % Convert the matrix with approximate decimal values (T3_real) to its
    % symbolic representation with 3 decimal places using VPA.
    T_real = vpa(T3_real, 3);
end
