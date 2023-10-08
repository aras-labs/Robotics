	
	Parallel Robots: Mechanics and Control
	Copyright Hamid D. Taghirad 2013
	Chapter Six
	Planar Cable Manipulator Motion Control

The programs used in this section are divided into the following directories:

1 PD Control   : Motion control with PD controller
2 FF Control   : Motion control with PD controller + Feedforward term
3 IDC          : Inverse Dynamics control with a PD controller
4 PIDC         : Inverse Dynamics control with partial linearization
5 Robust IDC   : Inverse Dynamics control + a robustifying term
6 Adaptive IDC : Inverse Dynamics control in an adaptive structure

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
	platform. Numerical redundancy resolution scheme is used in this 		routine

6-PlotData.m
	This program plots the simulation result.

7- SP_Perturbed: 
	This program perturbes the parameters of the model to examine 
	the robustness of the control scheme.

8-TP_cubic_s.m		
        This program generates a cubic trajectory for the manipulator.

The only subroutine which is different in each directory is

1-PD_Control.m
        This program generates the feedback forces exerted on the
        moving Platform by the PD control law 

2-FF_Control.m
        This program generates the feedback forces exerted on the
        moving  Platform by the PD control law + Feedforward term

3-IDC_Control.m
        This program generates the feedback forces exerted on the
        moving Platform by Inverse Dynamics control with a 
	PD controller

4-PIDC_control.m
	This program generates the feedback forces exerted on the
        moving Platform by Inverse Dynamics control with partial 			linearization

5-RIDC_Control.m
	This program generates the feedback forces exerted on the
        moving Platform by Inverse Dynamics control + a robustifying term

6-AIDC_Control.m 	
	This program generates the feedback forces exerted on the
        moving Platform by Inverse Dynamics control in an adaptive 			structure


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

To execute the program just run CL_Dynamic_Solver in Matlab. 

To change the parameters, and simulation condition just edit the 
Structural_Parameters.m file.