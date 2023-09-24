%% 
% 	An Introduction to Robotics, 
%   By Prof. Hamid D. Taghirad & Mohammad A. Khosravi
%   Copyright ARAS @2023
%
%   This program plots the results of different controllers.
%
load PD, load PID
load FF, load IDC 
load PIDC, load RIDC

figure(1)
subplot(211)
plot(PID.t,(PID.yd(1,:)-PID.y(1,:))*180/pi,'--k', ...
    FF.t,(FF.yd(1,:)-FF.y(1,:))*180/pi,'-.k', ...
    IDC.t,(IDC.yd(1,:)-IDC.y(1,:))*180/pi,'-k', ...
    PIDC.t,(PIDC.yd(1,:)-PIDC.y(1,:))*180/pi,':k', ...
    RIDC.t,(RIDC.yd(1,:)-RIDC.y(1,:))*180/pi,'-.b'),grid on
xlabel('time (sec)')
ylabel('e_1 (deg)')
title('Tracking Error')
legend('PID', 'FF','IDC','PIDC','RIDC','location','best')
subplot(212)
plot(PID.t,PID.tau(1,:),'--k', FF.t,FF.tau(1,:),'-.k', ...
    IDC.t,IDC.tau(1,:),'-k', PIDC.t,PIDC.tau(1,:),':k', ...
    RIDC.t,RIDC.tau(1,:),'-.b'),grid on
xlabel('time (sec)')
ylabel('\tau_1 (N.m)')
title('Control Effort')
  set(findall(figure(1),'type','line'),'linewidth',1.5)


figure(2)
subplot(211)
plot(PID.t,(PID.yd(2,:)-PID.y(2,:))*180/pi,'--k', ...
    FF.t,(FF.yd(2,:)-FF.y(2,:))*180/pi,'-.k', ...
    IDC.t,(IDC.yd(2,:)-IDC.y(2,:))*180/pi,'-k', ...
    PIDC.t,(PIDC.yd(2,:)-PIDC.y(2,:))*180/pi,':k', ...
    RIDC.t,(RIDC.yd(2,:)-RIDC.y(2,:))*180/pi,'-.b'),grid on
xlabel('time (sec)')
ylabel('e_2 (deg)')
title('Tracking Error')
legend('PID', 'FF','IDC','PIDC','RIDC','location','best')
subplot(212)
plot(PID.t,PID.tau(2,:),'--k', FF.t,FF.tau(2,:),'-.k', ...
    IDC.t,IDC.tau(2,:),'-k', PIDC.t,PIDC.tau(2,:),':k', ...
    RIDC.t,RIDC.tau(2,:),'-.b'),grid on
xlabel('time (sec)')
ylabel('\tau_2 (N.m)')
title('Control Effort')
  set(findall(figure(2),'type','line'),'linewidth',1.5)


 figure(3)
subplot(211)
plot(PID.t,(PID.yd(3,:)-PID.y(3,:))*180/pi,'--k', ...
    FF.t,(FF.yd(3,:)-FF.y(3,:))*180/pi,'-.k', ...
    IDC.t,(IDC.yd(3,:)-IDC.y(3,:))*180/pi,'-k', ...
    PIDC.t,(PIDC.yd(3,:)-PIDC.y(3,:))*180/pi,':k', ...
    RIDC.t,(RIDC.yd(3,:)-RIDC.y(3,:))*180/pi,'-.b'),grid on
xlabel('time (sec)')
ylabel('e_3 (deg)')
title('Tracking Error')
legend('PID', 'FF','IDC','PIDC','RIDC','location','best')
subplot(212)
plot(PID.t,PID.tau(3,:),'--k', FF.t,FF.tau(3,:),'-.k', ...
    IDC.t,IDC.tau(3,:),'-k',  PIDC.t,PIDC.tau(3,:),':k', ...
    RIDC.t,RIDC.tau(3,:),'-.b'),grid on
xlabel('time (sec)')
ylabel('\tau_3 (N.m)')
title('Control Effort')
  set(findall(figure(3),'type','line'),'linewidth',1.5)

return
