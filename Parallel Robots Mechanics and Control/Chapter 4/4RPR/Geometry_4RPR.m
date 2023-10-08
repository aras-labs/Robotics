function B=Geometry_4RRR(X,R,Bth);
%
%   Geometrical function of the 4RRR parallel manipulator
%
%   Input arguments:
%   X  : The position and orientation vector of the moving platform
%       X=[G,phi]' in which
%   G  : The position vector of the center of triangle:        2x1
%   phi: The angle of the triangle w.r.t horizontal line x:    1x1
%   R     : The radius of the circle of placement of Bi's to G 1x1
%   Bth   : The absolute angle of Bi at zero orientation       1x1
%
%   Output argument:
%   B     : The Position of Bi's                               2x4

phi=X(3);
for i=1:4,
    B(:,i)=[X(1)+R*cos(Bth(i)+phi); X(2)+R*sin(Bth(i)+phi)];
end

