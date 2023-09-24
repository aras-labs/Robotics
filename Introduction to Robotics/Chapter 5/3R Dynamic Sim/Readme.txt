	
	An Introduction to Robotics, 
    By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
    Copyright ARAS @2023

The following programs in this folder corresponds to Chapter 5 of the book: 

Main programs:
1- ID_sim.m	
    This code simulates the Inverse Dynamics of the 3R Robot
    
1- CL_sim.m	
    This code simulates the Closed-Loop Dynamics of the 3R Robot
    with PD controller
    
Other subroutines

1-Parameters.m  This fuction initializes the parameters of the 3R robot 
                in a Structure format
2-M_Matix       This function Defines 3R robot Mass matrix
3-G_Vector      This function Defines 3R robot Gravity vector
4-C_Matix       This function Defines 3R robot Christoffel matrix
5-PD_3R         This function Defines the pd control of the 3R robot
6-ID_3R         This function Defines the inverse dynamics of the 3R robot
7-TP_cubic      This function Defines cubic trejectory planner for 3R robot
