%% Attitude Kalman Filter - Assignment 10
% Name: Tanner Lex Jones

clear
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

% Load Mass Properties
mass_properties

%% Parameters
% Time parameters
tsim = 60; % simulation time in seconds
dt_sim = 0.001;
dt_gyros = 0.01; % Time between gyro measurements
dt_v1 = 1; % Time between vector 1 measurements
dt_v2 = 10; % Time between vector 2 measurements

% Noise parameters
sigmav1 = 1*pi/180; % STDev of noise on vector 1 measurements
sigmav2 = 0.1*pi/180; % STDev of noise on vector 2 measurements
sigmaw = 1*pi/180; % STDev of noise on gyro measurements
sigmab = 0.1*pi/180; % STDev of noise on gyro bias

% Inertial Vectors
v1_I = [1;0;0];
v2_I = [0;1;0];

%% Initial Conditions
% Initial rotation is 45deg about a [1;1;1] normalized axis
th = 45*pi/180;
e = [1;1;1]/norm([1;1;1]);
q0_BI = e2q(e, th);
A0_BI = q2A(q0_BI);
A0_IB = A0_BI';

% Initial angular rate is [-10;20;-30]deg/sec
wbi0_B = [-10; 20; -30]*pi/180; % rad/sec
Bw0 = [10;-20;-30]*pi/180; % initial bias of the gyro (rad/sec)
Bwhat0 = [0;0;0]; % initial estimate of gyro bias (rad/sec)

% Initial measurements
v1_B = A0_BI*v1_I + sigmav1*randn(3,1);
v2_B = A0_BI*v2_I + sigmav2*randn(3,1);

% Initial quaternion estimate
Ahat0_BI = triad(v2_B, v2_I, v1_B, v1_I);
qhat0_BI = A2q(Ahat0_BI);

% Initial covariance
sigmaangle = (180*pi/180)^2;
sigmaomega = (30*pi/180)^2;
P0 = diag([sigmaangle, sigmaangle, sigmaangle, ...
           sigmaomega, sigmaomega, sigmaomega]);

% seed values for random stuff
nb_seed = randi([0, 2^32], 3, 1);
nw_seed = randi([0, 2^32], 3, 1);
nv1_seed = randi([0, 2^32], 3, 1);
nv2_seed = randi([0, 2^32], 3, 1);


%% Simulation
data = sim('attitudeKalman', tsim);

%% Results

