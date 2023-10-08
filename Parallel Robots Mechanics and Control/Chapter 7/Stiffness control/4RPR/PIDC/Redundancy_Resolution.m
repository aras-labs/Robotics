%% 
%   Parallel Robots: Mechanics and Control
%   Copyright Hamid D. Taghirad 2013
%
%       This program generates the positive tension cable forces from
%	that generates the required cartesian forces to the moving 
%	platform. Numerical redundancy resolution scheme is used in this 		%	routine
%
function [SP,M_Dynamic_Mats] = Redundancy_Resolution(SP,KC,M_Dynamic_Mats)

M_Dynamic_Mats.JFP= zeros(SP.n,1) ;     % Positive Tension Actuator force

% Redundancy Resolution
options = optimset('Display', 'off','LargeScale','on','Algorithm',...
    'interior-point'); % Turn off Display
M_Dynamic_Mats.JFP=fmincon(@actnorm, SP.JFP_init ,[],[], ...
    KC.J',M_Dynamic_Mats.CF,SP.Fmin,[],[],options);
    SP.JFP_init= M_Dynamic_Mats.JFP;

    function V=actnorm(x)
        V=norm(x,2);
    end     % end of nested function
end         % end of main function