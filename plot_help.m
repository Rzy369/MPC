clear all

%% Loading the data

path_ti = load('data/path_ti.mat');
robot_ti = load('data/robot_ti.mat');

ourQR = load('data/ourQR.mat');

path = load('data/path.mat');

Q0_01 = load('data/Q0.01.mat');
Q0_1 = load('data/Q0.1.mat');
Q1 = load('data/Q1.mat');
Q10 = load('data/Q10.mat');

R0_01 = load('data/R0.01.mat');
R0_1 = load('data/R0.1.mat');
R1 = load('data/R1.mat');
R10 = load('data/R10.mat');
R100 = load('data/R100.mat');

N8 = load('data/N8.mat');
N20 = load('data/N20.mat');

%% Plotting the comparison between reference path and needle tip position for time_invariant system

t_ti = 0:0.1:9.9;
t_ti_robot = 0:0.1:10;

figure()
subplot(2,2,1);
plot(t_ti, path_ti.z_ref);
xlabel('time [s]');
ylabel('position [m]');
title('Z position of reference path', 'fontsize', 12);

subplot(2,2,2);
plot(t_ti_robot, robot_ti.z_robot);
xlabel('time [s]');
ylabel('position [m]');
title('Z position of needle tip', 'fontsize', 12);

subplot(2,2,3);
plot(t_ti, path_ti.gamma_ref);
xlabel('time [s]');
ylabel('angle [rad]');
title('\gamma angle of reference path', 'fontsize', 12);

subplot(2,2,4);
plot(t_ti_robot, robot_ti.gamma_robot);
xlabel('time [s]');
ylabel('angle [rad]');
title('\gamma angle of reference path', 'fontsize', 12);
sgtitle('\bf Comparison between reference path and needle tip when \kappa = 0', 'fontsize', 14)

%% Plotting the comparison between reference path and needle tip position for time_varying system
t_ref = 0:0.1:6
t_robot = 0:0.1:7

figure()
subplot(3,2,1);
plot(t_ref, path.x_ref);
xlabel('time [s]');
ylabel('position [mm]');
title('X position of reference path', 'fontsize', 12);

subplot(3,2,2);
plot(t_robot, ourQR.x_robot);
xlabel('time [s]');
ylabel('position [mm]');
xlim([0 6]);
title('X position of needle tip', 'fontsize', 12);

subplot(3,2,3);
plot(t_ref, path.y_ref);
xlabel('time [s]');
ylabel('position [cm]');
title('Y position of reference path', 'fontsize', 12);

subplot(3,2,4);
plot(t_robot, ourQR.y_robot);
xlabel('time [s]');
ylabel('position [cm]');
xlim([0 6]);
title('Y position of needle tip', 'fontsize', 12);

subplot(3,2,5);
plot(t_ref, path.z_ref);
xlabel('time [s]');
ylabel('position [mm]');
title('Z position of reference path', 'fontsize', 12);

subplot(3,2,6);
plot(t_robot, ourQR.z_robot);
xlabel('time [s]');
ylabel('position [mm]');
xlim([0 6]);
title('Z position of needle tip', 'fontsize', 12);
sgtitle('\bf Comparison between reference path and needle tip when \kappa \neq 0', 'fontsize', 14)

%% Plotting difference of Q

figure()
subplot(3,1,1)
plot(t_ref,path.x_ref); hold on;
plot(t_robot,ourQR.x_robot); hold on;
plot(t_robot,Q0_01.x_robot); hold on;
plot(t_robot,Q0_1.x_robot); hold on;
plot(t_robot,Q1.x_robot); hold on;
plot(t_robot,Q10.x_robot); hold on;
legend('ref path','ourQ','Q=0.01I','Q=0.1I','Q=I','Q=10I')
title('Position of x', 'fontsize', 12)
xlabel('time(s)')
ylabel('position(mm)')

subplot(3,1,2)
plot(t_ref,path.y_ref); hold on;
plot(t_robot,ourQR.y_robot); hold on;
plot(t_robot,Q0_01.y_robot); hold on;
plot(t_robot,Q0_1.y_robot); hold on;
plot(t_robot,Q1.y_robot); hold on;
plot(t_robot,Q10.y_robot); hold on;
title('Position of y', 'fontsize', 12)
xlabel('time(s)')
ylabel('position(cm)')


subplot(3,1,3)
plot(t_ref,path.z_ref); hold on;
plot(t_robot,ourQR.z_robot); hold on;
plot(t_robot,Q0_01.z_robot); hold on;
plot(t_robot,Q0_1.z_robot); hold on;
plot(t_robot,Q1.z_robot); hold on;
plot(t_robot,Q10.z_robot); hold on;
title('Position of z', 'fontsize', 12)
xlabel('time(s)')
ylabel('position(cm)')
sgtitle('\bf Comparison between different stage cost Q', 'fontsize', 14)
%% Plotting difference of Q

figure()
subplot(3,1,1)
plot(t_ref,path.x_ref); hold on;
plot(t_robot,ourQR.x_robot); hold on;
plot(t_robot,R0_01.x_robot); hold on;
plot(t_robot,R0_1.x_robot); hold on;
plot(t_robot,R1.x_robot); hold on;
plot(t_robot,R10.x_robot); hold on;
plot(t_robot,R100.x_robot); hold on;
legend('ref path','ourR','R=0.01I','R=0.1I','R=I','R=10I','R=100I')
title('Position of x', 'fontsize', 12)
xlabel('time(s)')
ylabel('position(mm)')

subplot(3,1,2)
plot(t_ref,path.y_ref); hold on;
plot(t_robot,ourQR.y_robot); hold on;
plot(t_robot,R0_01.y_robot); hold on;
plot(t_robot,R0_1.y_robot); hold on;
plot(t_robot,R1.y_robot); hold on;
plot(t_robot,R10.y_robot); hold on;
plot(t_robot,R100.y_robot); hold on;
title('Position of y', 'fontsize', 12)
xlabel('time(s)')
ylabel('position(cm)')


subplot(3,1,3)
plot(t_ref,path.z_ref); hold on;
plot(t_robot,ourQR.z_robot); hold on;
plot(t_robot,R0_01.z_robot); hold on;
plot(t_robot,R0_1.z_robot); hold on;
plot(t_robot,R1.z_robot); hold on;
plot(t_robot,R10.z_robot); hold on;
plot(t_robot,R100.z_robot); hold on;
title('Position of z', 'fontsize', 12)
xlabel('time(s)')
ylabel('position(cm)')
sgtitle('\bf Comparison between different stage cost R', 'fontsize', 14)

%% Plotting difference of horizon

figure()
subplot(3,1,1)
plot(t_ref,path.x_ref); hold on;
plot(t_robot,N8.x_robot, 'o'); hold on;
plot(t_robot,N20.x_robot); hold on;
legend('ref path','N=8','N=20')
title('Position of x', 'fontsize', 12)
xlabel('time(s)')
ylabel('position(mm)')

subplot(3,1,2)
plot(t_ref,path.y_ref); hold on;
plot(t_robot,N8.y_robot, 'o'); hold on;
plot(t_robot,N20.y_robot); hold on;
title('Position of y', 'fontsize', 12)
xlabel('time(s)')
ylabel('position(cm)')


subplot(3,1,3)
plot(t_ref,path.z_ref); hold on;
plot(t_robot,N8.z_robot, 'o'); hold on;
plot(t_robot,N20.z_robot); hold on;
title('Position of z', 'fontsize', 12)
xlabel('time(s)')
ylabel('position(cm)')
sgtitle('\bf Comparison between different horizons of the system', 'fontsize', 14);
