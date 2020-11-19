%% Attitude Dynamics Assignment
% Name: Tanner Lex Jones

%% Preliminaries
% This cleans all variables and sets the format to display more digits.
clearvars
close all
clc
format long

%% Addpath to Attitude Representations Folder
addpath('../01 Attitude Representations')

%% Addpath to Attitude Kinematics Folder
addpath('../02 Attitude Kinematics')

% Addpath to Attitude Dynamics Folder
addpath('../03 Attitude Dynamics')

%% Load qBus
load qBus.mat

%% Load Mass Properties
mass_properties

%% Display Mass Properties
disp('Js_S_B = ');
disp(Js_S_B);

disp('Jd_D_B = ');
disp(Jd_D_B);

disp('rcb_B = ');
disp(rcb_B);

disp('J_C_B = ');
disp(J_C_B);

disp('A_PB = ');
disp(A_PB);

disp('q_PB.s = ');
disp(q_PB.s);
disp('q_PB.v = ');
disp(q_PB.v);

disp('J_C_P = ');
disp(J_C_P);

%% Initial Satellite Attitude
% Create an initial attitude quaternion by rotating 45 degrees about an
% axis that points equally in the x, y, and z directions ([1;1;1]).
% Remember to make the axis a unit vector.
e = [1;1;1];  e = e/norm(e);
q0_BI = e2q(e,45*pi/180);
disp('q0_BI.s = ');
disp(q0_BI.s);
disp('q0_BI.v = ');
disp(q0_BI.v);

%%
% The initial attitude in the Simscape simulation can be specified by a
% Body to Inerital direction cosine matrix.  Convert q0_BI to A0_IB;
A0_BI = q2A(q0_BI);
A0_IB = A0_BI';
disp('A0_IB = ');
disp(A0_IB);

%% Initial Satellite Angular Velocity
% The same inital angular velocity will be used as in the attitude
% kinematics assignment.
wbi0_B = [5*pi/180;-10*pi/180;15*pi/180]; % rad/s

%% Run the Simulation
sim('dynamics',60)

%% Plot output of simulation
figure
plot(q_BI.s,'b')
hold on
plot(q_BI.v,'b')
plot(q_BI_simscape.s,'r--')
plot(q_BI_simscape.v,'r--')
title('Euler''s Equation and Simscape q^B_I','Interpreter','tex');
xlabel('seconds');
ylabel('q^B_I','Interpreter','tex');

figure
plot(q_BI.s - q_BI_simscape.s)
hold on
plot(q_BI.v - q_BI_simscape.v)
title('Difference between q^B_I','Interpreter','tex');
xlabel('seconds');
ylabel('q^B_I','Interpreter','tex');

figure
plot(wbi_B,'b')
hold on
plot(wbi_B_simscape,'r--')
title('Euler''s Equation and Simscape \omega_B_I^B','Interpreter','tex');
xlabel('seconds');
ylabel('radians/s');

figure
plot(wbi_B-wbi_B_simscape)
title('Difference between \omega_B_I^B','Interpreter','tex');
xlabel('seconds');
ylabel('radians/s');
