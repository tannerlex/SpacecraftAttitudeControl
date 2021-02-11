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

% Load dimensions
dimensions

%% Constants
uEarth = 3.986004418e14; % Earth Gravitational Parameter (m^3/s^2) C.104a
solarFlux = 1366; % Solar Flux (W/m^2)
c = 2.9979e8;
P_SR = solarFlux/c;

%% Initial Conditions
q0_BI.s = 1;
q0_BI.v = [0;0;0];

