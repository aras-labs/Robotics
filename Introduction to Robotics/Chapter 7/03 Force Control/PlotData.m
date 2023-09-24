%% 
% 	An Introduction to Robotics, 
%   By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
%   Copyright ARAS @2023
%
%       This program plots the simulation result.
%
function [Out] = PlotData(t,q,SP)

disp ('Recalculating the simulated data ...'); % Show the process time
tic
%
deg2rad = pi/180 ; rad2deg = 180/pi;
t=t' ; q=q';
%   Calculate the other system variables
for j=1:max(size(t));
[Fd(:,j),dFd(:,j),d2Fd(:,j)]=TP_quintic(t(j),SP);

Y(:,j)    = q(1:3,j) ; Ydot(:,j) = q(4:6,j) ;
Q(:,j)    = q(1:6,j) ; taui(:,j)   = q(7:9,j) ;
taudi(:,j)   = q(10:12,j);

[Kin] = Kinematics(Q(:,j), SP) ;
Jac=Kin.J;
taud(:,j) = Jac'*Fd(:,j);

[Dynamic_Mats] = Dynamic_Matrices(q(:,j), SP) ;
[Tau] = Force_Control(t(j),q(:,j),SP) ;

dist(:,j) = Tau.dist;
taue(:,j) = Tau.taue;   taum(:,j)= Tau.taum;
tau(:,j)=Tau.tau;       taut(:,j)=Tau.total;
FL(:,j)=Tau.FL;
M(:,:,j)=Dynamic_Mats.M; Fe(:,j)=Tau.Fe;
px(:,j)=Kin.x3; py(:,j)=Kin.y3; 
xrobot(:,j)=Kin.xrobot; yrobot(:,j)=Kin.yrobot;
xc(:,j)=Tau.xc; yc(:,j)=Tau.yc;
xe(:,j)=Kin.x3; ye(:,j)=Kin.y3;
a(:,j)=Tau.a; qa(:,j)=Tau.qa;

end
Out.t=t; Out.M=M; Out.y=q; Out.taud=taud;
Out.dist=dist;Out.a=a; Out.qa=qa;
Out.tau=tau; Out.taue=taue; Out.taut=taut;
Out.taum = taum; Out.Fe = Fe; Out.FL = FL;
Out.xc=xc; Out.yc=yc; Out.xe=xe; Out.ye=ye;
Out.xrobot=xrobot; Out.yrobot=yrobot; 
toc


figure(1)
i=max(size(t));deli=round(i/5);
i1=i-3*deli;i2=i-round(1.8*deli);
i3=i-round(1.1*deli);i4=i-round(0.5*deli);

yl=[0.8;1.3]; xl=SP.xl+(yl-SP.yl)/SP.sl;

plot(Out.xrobot(:,1),Out.yrobot(:,1),'-o',...
        Out.xrobot(:,i1),Out.yrobot(:,i1),'r-.o',...
        Out.xrobot(:,i2),Out.yrobot(:,i2),'g--o',...
        Out.xrobot(:,i3),Out.yrobot(:,i3),'m:o',...
        Out.xrobot(:,i4),Out.yrobot(:,i4),'c-o',...
        Out.xrobot(:,i),Out.yrobot(:,i),'k-.o',...
            xl,yl,'-.', ...
            Out.xe,Out.ye,'.r'), grid on
% , ...
%         Out.xc(:,1),Out.yc(:,1),'k+', ...
%         Out.xc(:,i1),Out.yc(:,i1),'k+', ...
%         Out.xc(:,i2),Out.yc(:,i2),'k+', ...
%         Out.xc(:,i3),Out.yc(:,i3),'k+', ...
%         Out.xc(:,i4),Out.yc(:,i4),'k+', ...
%         Out.xc(:,i),Out.yc(:,i),'k+'...
%         ),grid
% To check if the crossing point is correct!
    xlabel('x (meters)')
    ylabel('y (meters)')
  set(findall(figure(1),'type','line'),'linewidth',2)
% legend('state 1','state 2','state 3','state 4','state 5','state 6','location','best')


figure(2)
subplot(311)
plot(t,Out.taud(1,:),'k',t,Out.taum(1,:),':k'),grid on
xlabel('time (sec)')
ylabel('\tau_1 (N.m)')
title('Tracking Performance')
legend('\tau_d', '\tau_i')
subplot(312)
plot(t,Out.taud(2,:),'k',t,Out.taum(2,:),':k'),grid on
xlabel('time (sec)')
ylabel('\tau_2 (N.m)')
subplot(313)
plot(t,Out.taud(3,:),'k',t,Out.taum(3,:),':k'),grid on
xlabel('time (sec)')
ylabel('\tau_3 (N.m)')
set(findall(figure(2),'type','line'),'linewidth',1.5)


figure(3)
subplot(311)
plot(t,Out.taud(1,:)-Out.taum(1,:),'k'),grid on
title('Tracking Errors')
xlabel('time (sec)')
ylabel('\tau_1 (N.m)')
subplot(312)
plot(t,Out.taud(2,:)-Out.taum(2,:),'k'),grid on
xlabel('time (sec)')
ylabel('\tau_3 (N.m)')
subplot(313)
plot(t,Out.taud(3,:)-Out.taum(3,:),'k'),grid on
xlabel('time (sec)')
ylabel('\tau_3 (N.m)')
  set(findall(figure(3),'type','line'),'linewidth',1.5)

figure(4)
subplot(311)
plot(t,Out.tau(1,:),'k',t,Out.dist(1,:),':k', t,Out.taue(1,:),'-.k'), grid on
title('Actuator Torques');
xlabel('time (sec)')
ylabel('\tau_1 (N.m)')
legend('\tau','\tau_d', '\tau_e')
subplot(312)
plot(t,Out.tau(2,:),'k',t,Out.dist(2,:),':k', t,Out.taue(2,:),'-.k'), grid on
xlabel('time (sec)')
ylabel('\tau_2 (N.m)')
subplot(313)
plot(t,Out.tau(3,:),'k',t,Out.dist(3,:),':k', t,Out.taue(3,:),'-.k'), grid on
xlabel('time (sec)')
ylabel('\tau_3 (N.m)')
  set(findall(figure(4),'type','line'),'linewidth',1.5)

figure(5)
plot(t,Out.Fe(1,:),'k',t,Out.Fe(2,:),'k-.'),grid
title('Interacting Forces');
xlabel('time (sec)')
ylabel('Force (N)')
set(findall(figure(5),'type','line'),'linewidth',1.5)
legend('F_{e_x}','F_{e_y}','location','northwest')
 

return
