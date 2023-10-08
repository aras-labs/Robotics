	
	Parallel Robots: Mechanics and Control
	Copyright Hamid D. Taghirad 2013
	Chapter Five
	Stewart-Gough Manipulator Dynamic Analysis

The programs used in this section are as follows:

Main program:
1-Dynamic_Solver.m	
        This program solves the forward dynamics of 
        the Stewart-Gough platform using explicit formulation 
        for a typical initial conditions.

2-CL_Dynamic_Solver.m
        This program solves the inverse dynamics of 
        the Stewart-Gough platform using explicit formulation 
        in a closed-loop structure.

main sub routines:
1-Dynamic_Equation.m
        This program generates the explicit dynamic formulation
        of Stewart-Gough Platform

2-CL_Dynamic_Equation.m
        This program generates the closed-loop dynamic formulation
        of Stewart-Gough Platform

other subroutines
1-Structural_Parameters.m
        This program generates all the requires parameters of the 
        programs in a structure.

2-Kinematic_Configuration.m
        This program generates the kinematic configuration
        of Stewart-Gough Platform.

3-Parts_Dynamic_Matrixes.m
        This program generates the dynamic matrices of the compontnts
        of Stewart-Gough Platform.

4-Manipulator_Dynamic_Matrixes.m
        This program generates the total dynamic matrices 
        of Stewart-Gough Platform.

5-Control.m 
        This program is used for forward dynamics simulation and generates
        zero feedback force exerted on the Stewart-Gough Platform.

6-PD_Control.m
        This program generates the feedback forces exerted on the
        Stewart-Gough Platform by the PD control law 

7-TP_cubic_s.m		
        This program generates a cubic trajectory for the manipulator.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

To execute the program just run Dynamic_Solver (or CL_Dynamic_Solver)
in Matlab. To change the parameters, and siulation condition just edit the 
Structural_Parameters.m file.