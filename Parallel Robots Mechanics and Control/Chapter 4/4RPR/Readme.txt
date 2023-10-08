	
	Parallel Robots: Mechanics and Control
	Copyright Hamid D. Taghirad 2013
	Chapter Four
	Planar Cable Manipulator Jacobian Analysis

The programs used in this section are as follows:

Main program:

1-sensitivity.m		This program generates the singular points of 
                    the planar cable manipulator within its workspace.
                    Furthermore, it plots the sensitivity measure of 
                    the Jacobian for a typical trajectory.

2- stiffness.m      This program plots the reciprocal of the condition 
                    number of the planar cable manipulator within 
                    its workspace for four different constant orientations. 
                    Furthermore, it compares the stiffness values
                    for redundant and non-redundant structures.
   
Sub routines:
1-TP_cubic_s.m		This program generates a cubic trajectory for the 
                    manipulator.
2-Inv_Kin_4RRR.m	This program solves the inverse kinematics of the
        			planar cable manipulator
3- Geometry_4RRR.m	This program solves the geometry of the planar 
                    cable manipulator, and produce the required position 
                    vectors.
4-Jacobian.m        This program generates the Jacobian matrix
                    of the planar cable manipulator.
5-Jacobian_3.m      This program generates the Jacobian matrix
                    of the non-redundant planar cable manipulator.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

To execute the program just run sensitivity (or stiffness) function
in Matlab.
