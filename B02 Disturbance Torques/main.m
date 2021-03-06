%% Disturbance Torques - Assignment 2
% Name: Tanner Lex Jones

%% Preliminaries

% This cleans all variables and sets the format to display more digits.
clearvars
close all
clc
format long

% Addpath to Attitude Representations Folder
addpath('../01 Attitude Representations')

% Addpath to Attitude Kinematics Folder
addpath('../02 Attitude Kinematics')

% Addpath to Attitude Dynamics Folder
addpath('../03 Attitude Dynamics')

% Load qBus
load qBus.mat

% Load Mass Properties
mass_properties

% Load orbit sim data
load '../B01 Orbital Environment/orbit'
r_I = orbit.r_I;
v_I = orbit.v_I;
s_I = orbit.s_I;
sun = orbit.sun;
rho = orbit.rho;
B_I = orbit.B_I;

% Load dimensions
dimensions

%% Constants
uEarth = 3.986004418e14; % Earth Gravitational Parameter (m^3/s^2) C.104a
solarFlux = 1366; % Solar Flux (W/m^2)
c = 2.99792458e8; % Speed of Light
Psr = solarFlux/c; % Solar Radiation Pressure
M0_B = [0.001;-0.002;0.003]; % magnetic moment of spacecraft (A*m^2)

%% Initial Conditions
q0_BI.s = 1;
q0_BI.v = [0;0;0];


%% Simulate
disturbances = sim('disturbances', 3600);
save('disturbances.mat', 'disturbances');

%% Plot Results
figure
plot(disturbances.Tg_B);
title("Gravity Gradient Torque")
ylabel("Tg_B (Nm)")
figure
plot(disturbances.Ts_B);
title("Solar Radiation Torque")
ylabel("Ts_B (Nm)")
figure
plot(disturbances.Ta_B);
title("Aerodynamic Torque")
ylabel("Ta_B (Nm)")
figure
plot(disturbances.Tm_B);
title("Magnetic Disturbance Torque")
ylabel("Tm_B (Nm)")
