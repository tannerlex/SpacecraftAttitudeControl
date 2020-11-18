%% Angle Control Assignment
% Name: Tanner Lex Jones

%% Preliminaries
% This cleans all variables and sets the format to display more digits.
clearvars
close all
clc
format long

%%
% ctrlpref

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

%% Parameters
dt_delay = 0.01; % seconds

% amount of time to run simulation
t_sim = 0.1; % seconds

% Commanded angular velocity of the B frame relative to the I frame
% projected to the B frame.
wbistar_B = [1;1;1]; % radians/second

% Initial attitude
e = [1; 1; 1]; e = e/norm(e);
q0_BI = e2q(e, 45*pi/180);
% Desired Attitude
qstar_BI = e2q(e, 180*pi/180);

A0_BI = q2A(q0_BI);
A0_IB = A0_BI';

% Initial Satellite Angular Velocity
wbi0_B = [0; 0; 0];%*pi/180; % rad/s
wbi0_P = A_PB*wbi0_B;

% Define transfer function variable
s = tf('s');

%% The principal open loop plant models
G1 = 1/(J_C_P(1,1)*s);
display(G1);
G2 = 1/(J_C_P(2,2)*s);
display(G2);
G3 = 1/(J_C_P(3,3)*s);
display(G3);

%% 2. What is the open loop gain crossover frequency in the controller?
PM = 65*pi/180; % Phase Margin (rad)
DC_phase = -90*pi/180; % Phase at DC (rad)
% -pi = DC_phase - Phase Margin - Xover_angle
% Xover_angle = DC_phase -PM + pi
Xover_angle = DC_phase - PM + pi;

% crossover frequency
w_crossover = Xover_angle/dt_delay; % rad/s
display(w_crossover/(2*pi), 'Open Loop Gain Crossover Frequency (Hz)');

%% 3. What are the proportional control gains for each of the three axes?
% Design proportional control gains such that all three axes have identical
% responses.
Kd1 = 1/bode(G1, w_crossover);
display(Kd1);
Kd2 = 1/bode(G2, w_crossover);
display(Kd2);
Kd3 = 1/bode(G3, w_crossover);
display(Kd3);

% Control gains as diagonal matrix for input into simulink.
Kd = diag([Kd1; Kd2; Kd3]);

%% 4. Plot an Open Loop Bode Plot of each of the three axes with the controller in the loop
% Define the controller
C1 = tf(Kd1,'OutputDelay', dt_delay);
display(C1);
C2 = tf(Kd2,'OutputDelay', dt_delay);
display(C2);
C3 = tf(Kd3,'OutputDelay', dt_delay);
display(C3);

% figure
% bode(C1*G1, C2*G2, C3*G3, {1,1000}, '--');
% title('Open Loop Bode Plot');
% legend('1', '2', '3');

%% Display the phase margin and gain margin of each of the three controlled axes
[GM1, PM1] = margin(C1*G1);
display(mag2db(GM1), 'Gain Margin 1 (dB)');
display(PM1, 'Phase Margin 1 (degrees)');
[GM2, PM2] = margin(C2*G2);
display(mag2db(GM2), 'Gain Margin 2 (dB)');
display(PM2, 'Phase Margin 2 (degrees)');
[GM3, PM3] = margin(C3*G3);
display(mag2db(GM3), 'Gain Margin 3 (dB)');
display(PM3, 'Phase Margin 3 (degrees)');

%% Plot a Closed Loop Bode Plot for each of the three axes
CLTF1 = feedback(C1*G1, 1);
CLTF2 = feedback(C2*G2, 1);
CLTF3 = feedback(C3*G3, 1);
% figure
% bode(CLTF1, {1,1000})
% title('Closed Loop Bode Plot 1');
% figure
% bode(CLTF2, {1,1000})
% title('Closed Loop Bode Plot 2');
% figure
% bode(CLTF3, {1,1000})
% title('Closed Loop Bode Plot 3');

%% Step response information
stepinfo1 = stepinfo(CLTF1);
display(stepinfo1);
stepinfo2 = stepinfo(CLTF2);
display(stepinfo2);
stepinfo3 = stepinfo(CLTF3);
display(stepinfo3);

%% Outer Loop Control Design
[num, den] = pade(dt_delay, 8);
C_pade8 = tf(num, den);

config = sisoinit(6);
config.G1.value = G1;
config.C1.value = 1;
config.C2.value = Kd1*C_pade8;
config.G2.value = 1/s;
config.OL1.View = {'bode'};
config.OL2.View = {};
controlSystemDesigner(config);

Ko = 3356.8; % outer loop control gain
Zo = -0.01; % zero location (rad/s)
Po = -188.5; % pole location (rad/s)
Co = tf(Ko*(s-Zo)/(s-Po));
OLTF = Co*CLTF1/s;
bode(OLTF, {0.01, 1000})

%% Run the Simulation
sim('AngleControl',60)
