	
	An Introduction to Robotics, 
    By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
    Copyright ARAS @2023

The following folders corresponds to different controllers simulated 
on 3R robot and reported in Chapter 6 of the book: 

Folders:
PD Control      closed loop simulation of PD control on 3R robot
PID Control     closed loop simulation of PID control on 3R robot
FF Control      closed loop simulation of Feed forward + PID control on 3R robot
IDC Control     closed loop simulation of Inverse Dnamics + PID on 3R robot
PIDC Control    closed loop simulation of Partial IDC +PID on 3R robot
RIDC Control    closed loop simulation of Robust IDC +PID on 3R robot
Compare         Comparison of different closed loop simulations 

Programs in the folders:
1- CL_Dynamic_Solver    The main simulation code
2- CL_Dynamics          The main function called in the main program
3- Dynamic_Matrices     The Dynamic vector and matrices function of 3R robot
4- XXXX_Control         The controller function used in the closed loop simulation
5- PlotData             The function of plotting the results
6- TP_quintic           Trajectory Planner based on quintic polynomials

7- Structural_Parameters
                        All the simulation parameters are set in this 
                        function in a structural format

To run a new simulation, just change the simulation parameters in 
Structural_Parameters function and run CL_Dynamic_Solver on the right folder