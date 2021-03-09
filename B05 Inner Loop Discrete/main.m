%% Inner Loop Discrete Time Control - Assignment 5
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
s_rate = 100; Hz

% amount of time to run simulation
t_sim = 2; % seconds

% initial angular velocity
wbi0_B = [0;0;0];

% Commanded angular velocity of the B frame relative to the I frame
% projected to the B frame.
wbistar_B = [10;10;10]*pi/180; % radians/second

% Initial attitude
q0_BI.s = 1;
q0_BI.v = [0;0;0];

% Define transfer function variable
s = tf('s');

%% 1. What are each of the three principal open loop plants?
% Plant model
G1 = 1/(J_C_P(1,1)*s);
G2 = 1/(J_C_P(2,2)*s);
G3 = 1/(J_C_P(3,3)*s);
