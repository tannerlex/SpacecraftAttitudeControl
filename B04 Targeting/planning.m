%% Targeting - Assignment 4
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
Jmax = max(max(abs(J_C_P)));

% Load orbit sim data
load '../B01 Orbital Environment/orbit'
r_I = orbit.r_I;
v_I = orbit.v_I;
s_I = orbit.s_I;
sun = orbit.sun;
rho = orbit.rho;
B_I = orbit.B_I;
A_IE = orbit.A_IE; % DCM from Earth Fixed to Inertial

q0_BI.s = 1;
q0_BI.v = [0;0;0];

%%
% DCM to Deployable Panel
A_DB = euler2A(1,1,1,45*pi/180,0,0);

%%
sim('missionplanning', 2000);

