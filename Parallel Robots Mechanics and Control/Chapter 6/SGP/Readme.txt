	
	Parallel Robots: Mechanics and Control
	Copyright Hamid D. Taghirad 2013
	Chapter Six
	Stewart-Gough platform Motion Control

The programs used in this section are divided into the following directories:

1 PD Control   : Motion control with PD controller
2 FF Control   : Motion control with PD controller + Feedforward term
3 IDC          : Inverse Dynamics control with a PD controller
4 PIDC         : Inverse Dynamics control with partial linearization
5 Robust IDC   : Inverse Dynamics control + a robustifying term
6 Joint space Control : Control Schemes in joint space

The main function which are similar in each directories:

Main program:
1-CL_Dynamic_Solver.m
        This program solves the inverse dynamics of 
        the Stewart-Gough platform using explicit formulation 
        in a closed-loop structure.

main sub routines:
1-CL_Dynamic_Equation.m
        This program generates the closed-loop dynamic formulation
        of Stewart-Gough platform 

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

5-PlotData.m
	This program plots the simulation result.

7-SP_Perturbed: 
	This program perturbes the parameters of the model to examine 
	the robustness of the control scheme.

8-TP_cubic_s.m		
        This program generates a cubic trajectory for the manipulator.

The only subroutine which is different in each directory is

1-Control.m in 1 PD Control directory
        This program generates the feedback forces exerted on the
        moving Platform by the PD control law 

2-Control.m in 2 FF directory
        This program generates the feedback forces exerted on the
        moving  Platform by the PD control law + Feedforward term

3-Control.m in 3 IDC directory 
        This program generates the feedback forces exerted on the
        moving Platform by Inverse Dynamics control with a 
	PD controller

4-control.m in 4 PIDC directory 
	This program generates the feedback forces exerted on the
        moving Platform by Inverse Dynamics control with partial 
	linearization

5-Control.m in 5 Robust IDC directory 
	This program generates the feedback forces exerted on the
        moving Platform by Inverse Dynamics control + a robustifying term

6-Control.m  in Joint space control/1 PD directory	
	This program generates the feedback forces exerted on the
        moving Platform by PD control in joint space

7-Control.m  in Joint space control/2 FF directory	
	This program generates the feedback forces exerted on the
        moving Platform by Feedforward control in joint space

8-Control.m  in Joint space control/3 IDC directory	
	This program generates the feedback forces exerted on the
        moving Platform by IDC control in joint space

9-Control.m  in Joint space control/1 PD directory	
	This program generates the feedback forces exerted on the
        moving Platform by Partial IDC control in joint space


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

To execute the program just run CL_Dynamic_Solver in Matlab. 

To change the parameters, and simulation condition just edit the 
Structural_Parameters.m file.