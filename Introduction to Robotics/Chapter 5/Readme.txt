	
	An Introduction to Robotics, 
    By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
    Copyright ARAS @2023

The following programs in this folder corresponds to Chapter 5 of the book: 

Main programs:
1- Lagrange_XXXXX.m	
    This code determines the Dynamic Vectors and matrices of the
    Following robots using Lagrange method and Symbolic manipulation 
        2R       2R Robot        
        3R       3R Robot        
        SCARA    SCARA Manipulator        
 Elbow Folder    3DOF Elbow Manipulator        
 3R Dynamic Sim  Inverse Dynamics and Closed loop simulation of 3R robot
 2R Calibration  Calibration of 2R robot

2- Cubic_2R.m
    This code generates cubic trajectories for 2R robot

3- Cubic_2R_via.m
    This code generates cubic trajectories for 2R robot with via points

4- Quintic_2R.m
    This code generates quintic trajectories for 2R robot

5- blend_2R.m
    This code generates linear trajectories with parabolic blends for 2R robot

6- opt_blend_2R.m
    This code generates linear trajectories with parabolic blends 
    with optimal time manuvers for 2R robot

7- CubicE_2R.m
    This code generates cubic trajectories in task space for 2R robot

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

