%% 
% 	An Introduction to Robotics, 
%   By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
%   Copyright ARAS @2023
%
%   This program plots the simulation result.
%
function [Out] = PlotData(t,q,SP)

disp ('Recalculating the simulated data ...'); % Show the process time
tic
%
deg2rad = pi/180 ; rad2deg = 180/pi;
t=t' ; y=q';
%   Calculate the other system variables
for j=1:max(size(t));
[yd(:,j),dyd(:,j)]=TP_quintic(t(j),SP);
Y(:,j)    = y(1:3,j) ; Ydot(:,j) = y(4:6,j) ;
Q(:,j)    = y(1:6,j) ; Yi(:,j)   = y(7:9,j) ;
qi(:,j)   = y(10:12,j);
[Kin] = Kinematics(Q(:,j), SP) ;
[Dynamic_Mats] = Dynamic_Matrices(y(:,j), SP) ;
[Tau] = Stiffness_Control(t(j),y(:,j),SP) ;
PID(:,j) = Tau.PID;     taud(:,j) = Tau.taud;
taue(:,j) = Tau.taue;   FL(:,j)=Tau.FL;
tau(:,j)=Tau.tau;       taut(:,j)=Tau.total;
M(:,:,j)=Dynamic_Mats.M;
Fe(:,j)=Tau.Fe;
px(:,j)=Kin.x3; py(:,j)=Kin.y3; 
xrobot(:,j)=Kin.xrobot; yrobot(:,j)=Kin.yrobot;
xc(:,j)=Tau.xc; yc(:,j)=Tau.yc;
xe(:,j)=Kin.x3; ye(:,j)=Kin.y3;
deltay(:,j)=yd(1:2,j)- y(1:2,j)+taue(1:2,j)./[SP.Ke(1,1);SP.Ke(2,2)];
end
Out.t=t;Out.M=M;
Out.y=y; Out.yd=yd;
Out.PID=PID; Out.taud=taud;
Out.FL=FL; Out.tau=tau;
Out.taue=taue; Out.taut=taut;
Out.Fe=Fe; Out.xc=xc; Out.yc=yc;
Out.xe=xe; Out.ye=ye;
Out.xrobot=xrobot;
Out.yrobot=yrobot;
Out.deltay=deltay;
toc

figure(1)
subplot(311)
plot(t,yd(1,:)*rad2deg,'k',t,y(1,:)*rad2deg,':k'),grid on
xlabel('time (sec)')
ylabel('q_1 (deg)')
title('Tracking Performance')
legend('q_d', 'q_i')
subplot(312)
plot(t,yd(2,:)*rad2deg,'k',t,y(2,:)*rad2deg,':k','linewidth',1),grid on
xlabel('time (sec)')
ylabel('q_2 (deg)')
subplot(313)
plot(t,yd(3,:)*rad2deg,'k',t,y(3,:)*rad2deg,':k','linewidth',1),grid on
xlabel('time(sec)')
ylabel('q_3 (deg)')
set(findall(figure(1),'type','line'),'linewidth',1.5)


figure(2)
subplot(311)
plot(t,(yd(1,:)-y(1,:))*180/pi,'k'),grid on
title('Tracking Errors')
xlabel('time (sec)')
ylabel('e_1 (deg)')
%title([' The perturbation in the model parameters is %',num2str((1-Par.pert)*100)]);
subplot(312)
plot(t,(yd(2,:)-y(2,:))*180/pi,'k'),grid on
xlabel('time (sec)')
ylabel(['e_2 (deg)'])
subplot(313)
plot(t,(yd(3,:)-y(3,:))*180/pi,'k'),grid on
xlabel('time (sec)')
ylabel('e_3 (deg)')
  set(findall(figure(2),'type','line'),'linewidth',1.5)

figure(3)
subplot(311)
plot(t,tau(1,:),'k',t,taud(1,:),':k', t, taue(1,:),'-.k'), grid on
title('Actuator Torques');
xlabel('time (sec)')
ylabel('\tau_1 (N.m)')
legend('\tau','\tau_{d}', '\tau_e')
subplot(312)
plot(t,tau(2,:),'k',t,taud(2,:),':k', t, taue(2,:),'-.k'), grid on
xlabel('time (sec)')
ylabel('\tau_2 (N.m)')
subplot(313)
plot(t,tau(3,:),'k',t,taud(3,:),':k', t, taue(3,:),'-.k'), grid on
xlabel('time (sec)')
ylabel('\tau_3 (N.m)')
  set(findall(figure(3),'type','line'),'linewidth',1.5)

figure(4)
plot(t,Out.Fe(1,:)/1000,'k',t,Out.Fe(2,:)/1000,'k-.'),grid
title('Interacting Forces');
xlabel('time (sec)')
ylabel('Force (KN)')
set(findall(figure(4),'type','line'),'linewidth',1.5)
legend('F_{e_x}','F_{e_y}','location','best')
 
figure(5)
i=max(size(t));deli=round(i/5);
i1=i-4*deli;i2=i-3*deli;;i3=i-2*deli;i4=i-deli;
yl=[0.6;1.6]; xl=SP.xl+(yl-SP.yl)/SP.sl;
plot(Out.xrobot(:,1),Out.yrobot(:,1),'-o',...
        Out.xrobot(:,i1),Out.yrobot(:,i1),'r-.o',...
        Out.xrobot(:,i2),Out.yrobot(:,i2),'g--o',...
        Out.xrobot(:,i3),Out.yrobot(:,i3),'m:o',...
        Out.xrobot(:,i4),Out.yrobot(:,i4),'c-o',...
            Out.xrobot(:,i),Out.yrobot(:,i),'k-.o',...
            xl,yl,'-.', ...
            Out.xe,Out.ye,'--r'),grid
% To check if the crossing point is correct!
%
%         Out.xc(:,1),Out.yc(:,1),'k+', ...
%         Out.xc(:,i1),Out.yc(:,i1),'k+', ...
%         Out.xc(:,i2),Out.yc(:,i2),'k+', ...
%         Out.xc(:,i3),Out.yc(:,i3),'k+', ...
%         Out.xc(:,i4),Out.yc(:,i4),'k+', ...
%         Out.xc(:,i),Out.yc(:,i),'k+',...
    xlabel('x (meters)')
    ylabel('y (meters)')
  set(findall(figure(5),'type','line'),'linewidth',2)
 legend('state 1','state 2','state 3','state 4','state 5','state 6','location','best')

figure(6)
subplot(211)
plot(t,deltay(1,:),'k',t,taue(1,:)/1000,'-.k' )
title('First Joint')
xlabel('time (sec)')
ylabel('(rad) & (KN.m)')
legend('q_{d_1}-q_{e_1}','\tau_{e_1}')
subplot(212)
plot(t,deltay(2,:),'k',t,taue(2,:)/1000,'-.k' )
title('Second Joint')
xlabel('time (sec)')
ylabel('(rad) & (KN.m)')
legend('q_{d_2}-q_{e_2}','\tau_{e_2}')
set(findall(figure(6),'type','line'),'linewidth',1.5)

return
