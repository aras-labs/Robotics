	
	Parallel Robots: Mechanics and Control
	Copyright Hamid D. Taghirad 2013
	Chapter Seven
	Planar Cable Manipulator Force Control

The programs used in this section are divided into the following directories:

1-Stiffness Control: 
	Stiffness control with (partial) inverse dynamics control controller

2-Force Control: 
	Direct force control with (partial) inverse dynamics control controller

3-Impedance Control: 
	Impedance control with (partial) inverse dynamics control controller


The main function which are similar in each directories:

Main program:
1-CL_Dynamic_Solver.m
        This program solves the inverse dynamics of 
        the Planar Cable Manipulator using explicit formulation 
        in a closed-loop structure.

main sub routines:
1-CL_Dynamic_Equation.m
        This program generates the closed-loop dynamic formulation
        of Planar Cable Manipulator

other subroutines
1-Structural_Parameters.m
        This program generates all the requires parameters of the 
        programs in a structure.

2-Kinematic_Configuration.m
        This program generates the kinematic configuration
        of Planar Cable Manipulator.

3-Parts_Dynamic_Matrixes.m
        This program generates the dynamic matrices of the compontnts
        of Planar Cable Manipulator.

4-Manipulator_Dynamic_Matrixes.m
        This program generates the total dynamic matrices 
        of Planar Cable Manipulator.

5-Redundancy_Resolution.m
	This program generates the positive tension cable forces from
	that generates the required cartesian forces to the moving 
	platform. Numerical redundancy resolution scheme is used in this
	routine

6-PlotData.m
	This program plots the simulation result.

7- SP_Perturbed: 
	This program perturbes the parameters of the model to examine 
	the robustness of the control scheme.

8-TP_cubic_s.m		
        This program generates a cubic trajectory for the manipulator.

The only subroutine which is different in each directory is

1-SC_Control.m
        This program generates the feedback forces exerted on the
        moving Platform by the (partial) IDC Stiffness control 

2-FIDC_Control.m
        This program generates the feedback forces exerted on the
        moving Platform by the (partial) IDC direct force control 

3-IC_Control.m
        This program generates the feedback forces exerted on the
        moving Platform by the (partial) IDC Impedance control 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

To execute the program just run CL_Dynamic_Solver in Matlab. 

To change the parameters, and simulation condition just edit the 
Structural_Parameters.m file.