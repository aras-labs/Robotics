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
t=t' ; q=q';
%   Calculate the other system variables
for j=1:max(size(t));
[yd(:,j),dyd(:,j),d2yd(:,j)]=TP_quintic(t(j),SP);
 ys(:,j)=[yd(:,j);dyd(:,j);d2yd(:,j)];     % Augmented State
 qs(:,j)=IK(ys(:,j), SP);    qd(:,j) = qs(1:3,j);
 dqd(:,j)= qs(4:6,j);        d2qd(:,j)=qs(7:9,j);
 
Y(:,j)    = q(1:3,j) ; Ydot(:,j) = q(4:6,j) ;
Q(:,j)    = q(1:6,j) ; Yi(:,j)   = q(7:9,j) ;
qi(:,j)   = q(10:12,j);

[Kin] = Kinematics(Q(:,j), SP) ;
[Dynamic_Mats] = Dynamic_Matrices(q(:,j), SP) ;
[Tau] = Stiffness_Control(t(j),q(:,j),SP) ;

PID(:,j) = Tau.PID;     taud(:,j) = Tau.taud;
taue(:,j) = Tau.taue;   FL(:,j)=Tau.FL;
tau(:,j)=Tau.tau;       taut(:,j)=Tau.total;
M(:,:,j)=Dynamic_Mats.M; Fe(:,j)=Tau.Fe;
px(:,j)=Kin.x3; py(:,j)=Kin.y3; 
xrobot(:,j)=Kin.xrobot; yrobot(:,j)=Kin.yrobot;
xc(:,j)=Tau.xc; yc(:,j)=Tau.yc;
xe(:,j)=Kin.x3; ye(:,j)=Kin.y3;

end
Out.t=t;Out.M=M; Out.y=q; Out.yd=yd; Out.PID=PID; Out.taud=taud;
Out.FL=FL; Out.tau=tau; Out.taue=taue; Out.taut=taut;
Out.Fe=Fe; Out.xc=xc; Out.yc=yc; Out.xe=xe; Out.ye=ye;
Out.xrobot=xrobot; Out.yrobot=yrobot; 
toc

figure(1)
subplot(311)
plot(t,qd(1,:)*rad2deg,'k',t,q(1,:)*rad2deg,':k'),grid on
xlabel('time (sec)')
ylabel('q_1 (deg)')
title('Tracking Performance')
legend('q_d', 'q_i')
subplot(312)
plot(t,qd(2,:)*rad2deg,'k',t,q(2,:)*rad2deg,':k','linewidth',1),grid on
xlabel('time (sec)')
ylabel('q_2 (deg)')
subplot(313)
plot(t,qd(3,:)*rad2deg,'k',t,q(3,:)*rad2deg,':k','linewidth',1),grid on
xlabel('time(sec)')
ylabel('q_3 (deg)')
set(findall(figure(1),'type','line'),'linewidth',1.5)


figure(2)
subplot(311)
plot(t,(qd(1,:)-q(1,:))*180/pi,'k'),grid on
title('Tracking Errors')
xlabel('time (sec)')
ylabel('e_1 (deg)')
subplot(312)
plot(t,(qd(2,:)-q(2,:))*180/pi,'k'),grid on
xlabel('time (sec)')
ylabel(['e_2 (deg)'])
subplot(313)
plot(t,(qd(3,:)-q(3,:))*180/pi,'k'),grid on
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
plot(t,Out.Fe(1,:),'k',t,Out.Fe(2,:),'k-.'),grid
title('Interacting Forces');
xlabel('time (sec)')
ylabel('Force (N)')
set(findall(figure(4),'type','line'),'linewidth',1.5)
legend('F_{e_x}','F_{e_y}')
 
figure(5)
i=max(size(t));deli=round(i/5);
i1=i-3*deli;i2=i-round(2.5*deli);
i3=i-round(2.2*deli);i4=i-round(1.5*deli);
%i1=i-140;i2=i-120;i3=i-100;i4=i-50;        %For better show
yl=[0.4;1.2]; xl=SP.xl+(yl-SP.yl)/SP.sl;
yd=[SP.xd(1,3); SP.xd(3,3)]; 
xd=SP.xd(1,2)+(yd-SP.xd(1,3))/SP.sl;

plot(Out.xrobot(:,1),Out.yrobot(:,1),'-o',...
        Out.xrobot(:,i1),Out.yrobot(:,i1),'r-.o',...
        Out.xrobot(:,i2),Out.yrobot(:,i2),'g--o',...
        Out.xrobot(:,i3),Out.yrobot(:,i3),'m:o',...
        Out.xrobot(:,i4),Out.yrobot(:,i4),'c-o',...
        Out.xrobot(:,i),Out.yrobot(:,i),'k-.o',...
            xl,yl,'-.', xd,yd,'-.',...
            Out.xe,Out.ye,'.r', ...
        Out.xc(:,1),Out.yc(:,1),'k+', ...
        Out.xc(:,i1),Out.yc(:,i1),'k+', ...
        Out.xc(:,i2),Out.yc(:,i2),'k+', ...
        Out.xc(:,i3),Out.yc(:,i3),'k+', ...
        Out.xc(:,i4),Out.yc(:,i4),'k+', ...
        Out.xc(:,i),Out.yc(:,i),'k+'...
        ),grid
% To check if the crossing point is correct!

    xlabel('x (meters)')
    ylabel('y (meters)')
  set(findall(figure(5),'type','line'),'linewidth',2)
% legend('state 1','state 2','state 3','state 4','state 5','state 6','location','best')

return
