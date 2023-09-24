	
	An Introduction to Robotics, 
    By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
    Copyright ARAS @2023

The following programs in this folder corresponds to Chapter 4 of the book: 

Main programs:
1- Jacobian_XXXXX.m	
    This code determines the Jacobian matrix of the 
    RRR       		3R Manipulator        General and Screw-based
    RRR_inspection   	3R Manipulator        By inspection
    ElbowSix  		6DOF Elbow robot      Screw-based
    SCARA     		SCARA Manipulator     General
    SCARA_screw		SCARA Manipulator     Screw-based	
    Stanford 		Stanford Manipulator  Screw-based in two frames

2- twoRisotropy.m
    This code provides singuarity analysis of 2R robot

3- Obstacle_3R.m (in the book ammendments)
    Ths code solves optimal redundancy resolution for a 3R robot 
    for obstacle avoidance objective

4- Singularity_3R.m (in the book ammendments)
    Ths code solves optimal redundancy resolution for a 3R robot 
    for Singularity Circumnetioan objective

Other subroutines

1-DH.m      This function Defines DH Parameters
2-SR.m      This function Defines Screw Parameters
3-Vector_Vpa.m 
        This code provides a better representation of the 
        symbolic vectors in MATLAB
4-Matrix_Vpa.m
        This code provides a better representation of the
         symbolic matrices in MATLAB
5-Matrix_Vpa_Numeric.m		
        This code provides a better representation of
        symbolic matrices that are numerically evaluated.

