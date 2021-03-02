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

%% Parameters
dt_delay = 0.01; % seconds

% amount of time to run simulation
t_sim = 3600; % seconds

%% Reaction Wheel Properties
wn = 2*pi*10; % Reaction Wheel Natural Frequency
zeta = sqrt(2)/2; % Reaction Wheel Damping Ratio
hwmax = 0.015; % Nms
hwdotmax = 0.004; % Nm
safety = 0.5; % reaction wheel safety factor
wmax = safety*hwmax/Jmax;
wdotmax = safety*hwdotmax/Jmax;

%% Mission Planning: determine pointing angles throughout mission
planning

