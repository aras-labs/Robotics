function [] = PlotStructure(t,X,SP)

A = SP.A ;
figure;
hold ;
grid;
axis([-4 4 -4 4 -4 1]);
view(135,45);
l = zeros(SP.n) ;
bl = zeros(SP.n) ;

for j = 1:SP.n 
    l(j) = line([0 0],[0 0],[0 0]) ;
    bl(j) = line([0 0],[0 0],[0 0],'color',[0 1 0]) ;    
    if j < SP.n 
        line([A(1,j) A(1,j+1)],[A(2,j) A(2,j+1)],[A(3,j) A(3,j+1)],'color',[1 0 0]) ;
    else
        line([A(1,1) A(1,j)],[A(2,1) A(2,j)],[A(3,1) A(3,j)],'color',[1 0 0]) ;
    end 
end


for i=1:size(t,1)
    
    x = X(i,1:3) ; % Position Vector of the Origin of the Moving Coordinate System relative to Fixed Coordinate System (m)
    phi = X(i,4:6) ; % Column Matrix of Euler angels; Orientation of E.F. (rad)
    alpha = phi(1) ; % Euler Angle alpha (rad)
    beta = phi(2) ; % Euler Angle beta (rad)
    gamma = phi(3) ; % Euler Angle gamma (rad)
    R = [cos(beta)*cos(gamma) cos(gamma)*sin(alpha)*sin(beta)-cos(alpha)*sin(gamma) cos(alpha)*cos(gamma)*sin(beta)+sin(alpha)*sin(gamma)
         cos(beta)*sin(gamma) cos(alpha)*cos(gamma)+sin(alpha)*sin(beta)*sin(gamma) -cos(gamma)*sin(alpha)+cos(alpha)*sin(beta)*sin(gamma)
          -sin(beta)                            cos(beta)*sin(alpha)                                      cos(alpha)*cos(beta)            ]; % Rotation Matrix Form B to A
    B = zeros(3,SP.n) ; % Position of Moving Attachment Points in Fixed Coordinate System (m)
    for j=1:SP.n       
        B(:,j) = x'+R*SP.B_B(:,j) ;
    end
    for j=1:SP.n               
        set(l(j),'xdata',[A(1,j) B(1,j)]) ;
        set(l(j),'ydata',[A(2,j) B(2,j)]) ;
        set(l(j),'zdata',[A(3,j) B(3,j)]) ;
        if j < SP.n 
            set(bl(j),'xdata',[B(1,j) B(1,j+1)]) ;
            set(bl(j),'ydata',[B(2,j) B(2,j+1)]) ;
            set(bl(j),'zdata',[B(3,j) B(3,j+1)]) ;
        else
            set(bl(j),'xdata',[B(1,1) B(1,j)]) ;
            set(bl(j),'ydata',[B(2,1) B(2,j)]) ;
            set(bl(j),'zdata',[B(3,1) B(3,j)]) ;            
        end 
    end
    M(i) = getframe(gcf);
end

hold ;

clf ;
axes('Position',[0 0 1 1]);
%movie(M,10) ;

return
