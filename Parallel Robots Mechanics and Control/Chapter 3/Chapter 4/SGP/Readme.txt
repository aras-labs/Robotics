	
	Parallel Robots: Mechanics and Control
	Copyright Hamid D. Taghirad 2013
	Chapter Four
	Stewart-Gough Manipulator

The programs used in this section are as follows:

Main program:
1-Singularity_SGP_Z.m	
	This program generates the singularity configurations 
    of the Stewart-Gough Platform for constant z workspace

2-Singularity_SGP_theta.m	
	This program generates the singularity configurations 
    of the Stewart-Gough Platform for constant orientation workspace

3-Stiffness_SGP.m	
	This program plots the reciprocal of the condition number of the 
    Jacobian matrix for constant orientations workspace.
    Furthermore, it plots the Maximum singular value of the 
    Jacobian matrix over the constant orientation workspace. 

Sub routines:
1-Jacobian
        This program generates Jacobian Matrix of the Stewart Gough
        Platform.
2-sc2rot
        This program generates the rotation matrix from screw coordinates.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

To execute the program just run Stiffness_SGP (or other main programs)
in Matlab.
