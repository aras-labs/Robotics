	
	An Introduction to Robotics, 
    By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
    Copyright ARAS @2023

The following programs in this folder corresponds to Chapter 5 of the book: 

Main programs:
1- CL_sim.m	
    This code simulates the Closed-Loop Dynamics of the 2R Robot
    with PD controller and dervies the Calibration Parameters
    
Other subroutines

1-Parameters.m  This fuction initializes the parameters of the 2R robot 
                in a Structure format
2-M_Matix       This function Defines 2R robot Mass matrix
3-G_Vector      This function Defines 2R robot Gravity vector
4-C_Matix       This function Defines 2R robot Christoffel matrix
5-PD_2R         This function Defines the pd control of the 2R robot
6-TP_cubic      This function Defines cubic trejectory planner for 2R robot

Data Files:

Test.mat        Calibration parameters listed in a table

