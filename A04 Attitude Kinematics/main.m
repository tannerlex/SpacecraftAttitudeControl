%% Attitude Kinematics Assignment
% Name: Tanner Lex Jones

%% Preliminaries
% This cleans all variables and sets the format to display more digits.
clearvars
close all
clc
format long

%% Load Quaternion Bus Into Workspace
load qBus.mat

%% Addpath to Attitude Representations Folder
addpath('../01 Attitude Representations')

%% Initial Satellite Attitude
% Create initial axis and angle by rotating 45 degrees about z axis
e0 = [0;0;1];
angle0 = 45*pi/180;

%%
% Convert inital Eigen Axis/Angle to Quaternion
q0_BI = e2q(e0, angle0);

%%
% Convert initial Eigen Axis/Angle to Rotation Vector
theta0_BI = e0*angle0;

%%
% Convert initial Eigen Axis/Angle to DCM
A0_BI = e2A(e0,angle0);

%%
% Convert initial DCM to ZYX Euler Angles and place in a vector
[phi, theta, psi] = A2zyx(A0_BI);
zyx0 = [phi;theta;psi];

%% Satellite Angular Velocity
wbi0_B = [5*pi/180;-10*pi/180;15*pi/180]; %radians/second

%% Run the Simulation
sim('kinematics',60)

%% Plot phi
figure
plot(tout,180/pi*[phi_quat,phi_DCM,phi_zyx,phi_rotvec,phi_true]);
title('\phi');
xlabel('seconds');
ylabel('degrees');
legend('\phi_q_u_a_t','\phi_D_C_M','\phi_z_y_x','\phi_r_o_t_v_e_c','\phi_t_r_u_e');

%% Plot phi difference from true
figure
plot(tout,180/pi*([phi_quat,phi_DCM,phi_zyx,phi_rotvec]-phi_true));
title('\phi difference from true');
xlabel('seconds');
ylabel('degrees');
legend('\phi_q_u_a_t','\phi_D_C_M','\phi_z_y_x','\phi_r_o_t_v_e_c');

%% Plot theta
figure
plot(tout,180/pi*[theta_quat,theta_DCM,theta_zyx,theta_rotvec,theta_true]);
title('\theta');
xlabel('seconds');
ylabel('degrees');
legend('\theta_q_u_a_t','\theta_D_C_M','\theta_z_y_x','\theta_r_o_t_v_e_c','\theta_t_r_u_e');

%% Plot theta difference from true
figure
plot(tout,180/pi*([theta_quat,theta_DCM,theta_zyx,theta_rotvec]-theta_true));
title('\theta difference from true');
xlabel('seconds');
ylabel('degrees');
legend('\theta_q_u_a_t','\theta_D_C_M','\theta_z_y_x','\theta_r_o_t_v_e_c');

%% Plot psi
figure
plot(tout,180/pi*[psi_quat,psi_DCM,psi_zyx,psi_rotvec,psi_true]);
title('\psi');
xlabel('seconds');
ylabel('degrees');
legend('\psi_q_u_a_t','\psi_D_C_M','\psi_z_y_x','\psi_r_o_t_v_e_c','\psi_t_r_u_e');

%% Plot psi difference from true
figure
plot(tout,180/pi*([psi_quat,psi_DCM,psi_zyx,psi_rotvec]-psi_true));
title('\psi difference from true');
xlabel('seconds');
ylabel('degrees');
legend('\psi_q_u_a_t','\psi_D_C_M','\psi_z_y_x','\psi_r_o_t_v_e_c');