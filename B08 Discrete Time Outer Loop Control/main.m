%% Raspberry Pi Inner Loop Control - Assignment 7
% Name: Tanner Lex Jones

%% Preliminaries

% This cleans all variables and sets the format to display more digits.
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
t_sim = 1; % seconds
dt_delay = 0.011; % seconds
dt_sample = 0.01; % seconds

% initial angular velocity
wbi0_B = [0;0;0];

% Initial attitude
e = [1; 1; 1]/norm([1; 1; 1]); % rotation axis
th = 0*pi/180; % rotation angle
q0_BI = e2q(e, th); % initial attitude

% Desired attitude
e = [1; 1; 1]/norm([1; 1; 1]); % rotation axis
th = 180*pi/180; % rotation angle
qstar_BI = e2q(e, th); % desired attitude


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

% redesign using 0.011 second delay
% controlSystemDesigner(G1d/z);

% chosen for 60deg phase margin and equal response for each axis:
K1d = 0.2235; % C1 = 0.2235 gives PM 60deg at 5.04Hz, GM 9.2dB at 15.2Hz
K2d = 1.214; % C2 = 1.214 gives PM 60deg at 5.04Hz, GM 9.2dB at 15.2Hz
K3d = 1.2589; % C3 = 1.2589 gives PM 60deg at 5.04Hz, GM 9.2dB at 15.2Hz

% Control gains as diagonal matrix for input into simulink.
Kd = diag([K1d; K2d; K3d]);

%% Closed Loop Transfer Functions

% closed loop transfer function bode plots
CLTF1d = feedback(K1d*G1d, 1);
CLTF2d = feedback(K2d*G2d, 1);
CLTF3d = feedback(K3d*G3d, 1);


%% Outer loop control design

% discrete outer loop integrator
Gd = c2d(1/s,dt_sample,'foh');

% discrete time representation of time delay
Cdelay = c2d(exp(-s*dt_delay),dt_sample,'zoh');

C1d = K1d*Cdelay;
config = sisoinit(6);
config.G1.value = G1d;
config.C2.value = C1d;
config.G2.value = Gd;
config.C1.value = 1;
config.OL1.View = {'bode'};
config.OL2.View = {};
% controlSystemDesigner(config);

wp = 2*pi*11;
wz = 2*pi*0.000955;
w_crossover = 2*pi*1.57;

zd = exp(-wz*dt_sample);
pd = exp(-wp*dt_sample);

Cd = 1/(z-1)*(z-zd)/(z-pd);
K = 1/bode(Cd*CLTF1d*Gd,w_crossover);


%% Simulate simulink model
sim('discreteOuter',t_sim)

%% Time Delay
% The following was used to determine the time delay of 0.011 seconds:
dclk = squeeze(d_clock.Data);
plot(d_clock.time,dclk)
title('Time Delay')
xlabel('sample')
ylabel('delay (seconds)')


