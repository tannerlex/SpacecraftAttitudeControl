%% Raspberry Pi Inner Loop Control - Assignment 7
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


%% Parameters
dt_delay = 0.01; % seconds
dt_sample = 0.01;

% initial angular velocity
wbi0_B = [0;0;0];

% Commanded angular velocity of the B frame relative to the I frame
% projected to the B frame.
wbistar_B = [0;0;0]*pi/180; % radians/second

% Initial attitude
q0_BI.s = 1;
q0_BI.v = [0;0;0];


%% Plant models

% Define transfer function variables
s = tf('s');
z = tf('z');

G1 = 1/(J_C_P(1,1)*s);
G2 = 1/(J_C_P(2,2)*s);
G3 = 1/(J_C_P(3,3)*s);

% convert to discrete time
G1d = c2d(G1, dt_sample, 'zoh');
G2d = c2d(G2, dt_sample, 'zoh');
G3d = c2d(G3, dt_sample, 'zoh');


%% Control design

% controlSystemDesigner(G3d/z);

% chosen for 60deg phase margin
K1d = 0.246167;
K2d = 1.337558;
K3d = 1.38698;

% Control gains as diagonal matrix for input into simulink.
Kd = diag([K1d; K2d; K3d]);

