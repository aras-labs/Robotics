% Conventional Jacobian for RRR robot
% By inspection (about frame{0})
% This is a comment explaining the purpose of the code.


% Clears all variables from the workspace. This ensures there are no conflicts or residues from previous runs.
clear all

% Declares symbolic variables a1, a2, a3, th1, th2, and th3. These variables will be used to represent parameters of the robot.
syms('a1','a2','a3', 'th1','th2','th3')

% Creates the 3x3 rotation matrix R01 based on the first joint angle th1.
R01=[cos(th1) -sin(th1) 0
    sin(th1) cos(th1) 0
    0 0 1];

% Creates the 3x3 rotation matrix R12 based on the second joint angle th2.
R12=[cos(th2) -sin(th2) 0
    sin(th2) cos(th2) 0
    0 0 1];

% Creates the 3x3 rotation matrix R23 based on the third joint angle th3.
R23=[cos(th3) -sin(th3) 0
    sin(th3) cos(th3) 0
    0 0 1];

% Multiplies the rotation matrices R01 and R12 and simplifies the result to get R02.
R02=simplify(R01*R12);

% Multiplies the rotation matrices R02 and R23 and simplifies the result to get R03.
R03=simplify(R02*R23);

% Computes the inverse of R01 and simplifies the result to get R10.
R10=simplify(inv(R01));

% Computes the inverse of R12 and simplifies the result to get R21.
R21=simplify(inv(R12));

% Computes the inverse of R02 and simplifies the result to get R20.
R20=simplify(inv(R02));

% Computes the inverse of R03 and simplifies the result to get R30.
R30=simplify(inv(R03));

% Defines the unit vectors s1, s2, and s3 along the z-axes of frames 1, 2, and 3, respectively.
s1=[0;0;1]; s2=s1; s3=s1;

% Computes the vector so3 representing the position of the origin of frame 3 with respect to frame 0.
so3=R03*[-a3;0;0];

% Computes the vector so2 representing the position of the origin of frame 2 with respect to frame 0.
so2=so3+R02*[-a2;0;0];

% Computes the vector so1 representing the position of the origin of frame 1 with respect to frame 0.
so1=so2+R01*[-a1;0;0];

% Computes the cross product of so1 and s1 to obtain the first column of the Jacobian matrix J1.
J1=[cross(so1,s1)];

% Computes the cross product of so2 and s2 to obtain the second column of the Jacobian matrix J2.
J2=[cross(so2,s2)];

% Computes the cross product of so3 and s3 to obtain the third column of the Jacobian matrix J3.
J3=[cross(so3,s3)];

% Combines the s1, s2, and s3 vectors along with their respective Jacobian columns J1, J2, and J3 to form the Jacobian matrix J00.
J00=[s1 s2 s3 
    J1 J2 J3];

% Re-defines the unit vectors s1, s2, and s3 along the z-axes of frames 1, 2, and 3, respectively. (This seems redundant.)
s1=[0;0;1]; s2=s1; s3=s1;

% Resets the vector so3 to represent the position of the origin of frame 3 with respect to frame 0 as the zero vector.
so3=[0;0;0];

% Computes the vector so2 representing the position of the origin of frame 2 with respect to frame 0.
so2=so3+R02*[-a2;0;0];

% Computes the vector so1 representing the position of the origin of frame 1 with respect to frame 0.
so1=so2+R01*[-a1;0;0];

% Computes the cross product of so1 and s1 to obtain the first column of the Jacobian matrix J1.
J1=[cross(so1,s1)];

% Computes the cross product of so2 and s2 to obtain the second column of the Jacobian matrix J2.
J2=[cross(so2,s2)];

% Computes the cross product of so3 and s3 to obtain the third column of the Jacobian matrix J3.
J3=[cross(so3,s3)];

% Combines the s1, s2, and s3 vectors along with their respective Jacobian columns J1, J2, and J3 to form the Jacobian matrix J02.
J02=[s1 s2 s3 
    J1 J2 J3];