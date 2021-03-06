%% Targeting - Assignment 4
% Name: Tanner Lex Jones

%% Preliminaries

% This cleans all variables and sets the format to display more digits.
clearvars
close all
clc
format long

% Add path to Attitude Representations Folder
addpath('../01 Attitude Representations')

% Add path to Attitude Kinematics Folder
addpath('../02 Attitude Kinematics')

% Add path to Attitude Dynamics Folder
addpath('../03 Attitude Dynamics')

% Load qBus
load qBus.mat

%% Load Mass Properties
mass_properties
Jmax = max(max(abs(J_C_P)));

% DCM to Deployable Panel
A_DB = euler2A(1,1,1,45*pi/180,0,0);

%% Reaction Wheel Transfer Function
wn = 2*pi*10; % Reaction Wheel Natural Frequency
zeta = sqrt(2)/2; % Reaction Wheel Damping Ratio
hwmax = 0.015; % Nms
hwdotmax = 0.004; % Nm
safety = 0.5; % reaction wheel safety factor
wmax = safety*hwmax/Jmax;
wdotmax = safety*hwdotmax/Jmax;

% Define transfer function variable
s = tf('s');

Gw = wn^2/(s^2 + 2*zeta*wn*s + wn^2);

%% Parameters
dt_delay = 0.01; % seconds

% amount of time to run simulation
t_sim = 3600; % seconds


%% Mission Planning: determine pointing angles throughout mission
planning

%% Initial conditions

% initial attitude
q0_BI.s = qs_BI.s.data(1);
q0_BI.v = qs_BI.v.data(1,:)';
A0_BI = q2A(q0_BI);
A0_IB = A0_BI';

wbi0_B = [0;0;0]; % Initial Satellite Angular Velocity rad/s
wbi0_P = A_PB*wbi0_B;
hw0_B = [0;0;0]; % Initial Reaction Wheel Angular Momentum Nms

%% Controller Design
controlDesign

%% Run Simulation
% sim('SatelliteAttitudeTargeting',t_sim)
sim('SatelliteAttitudeTargeting',1)
