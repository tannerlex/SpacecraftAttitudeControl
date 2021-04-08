%% Raspberry Pi Outer Loop Control - Assignment 8
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

% define the outer loop controller
Cd = 1/(z-1)*(z-zd)/(z-pd);
K = 1/bode(Cd*CLTF1d*Gd,w_crossover);
Cd = K*Cd;


%% Outer loop design analysis
Outer_OLTF1 = Cd*CLTF1d*Gd;
Outer_OLTF2 = Cd*CLTF2d*Gd;
Outer_OLTF3 = Cd*CLTF3d*Gd;
figure
bode(Outer_OLTF1, Outer_OLTF2, Outer_OLTF3, {0.001, 1000}, '--');
legend('1', '2', '3');
title('Open Outer Loop Bode Plot');

[GMO1, PMO1] = margin(Outer_OLTF1);
display(mag2db(GMO1), 'Gain Margin 1 (dB)');
display(PMO1, 'Phase Margin 1 (degrees)');

Outer_CLTF1 = feedback(Outer_OLTF1,1);
Outer_CLTF2 = feedback(Outer_OLTF2,1);
Outer_CLTF3 = feedback(Outer_OLTF3,1);
figure
bode(Outer_CLTF1, Outer_CLTF2, Outer_CLTF3, {0.01, 1000}, '--');
legend('1', '2', '3');
title('Closed Outer Loop Bode Plot');

Outer_stepinfo1 = stepinfo(Outer_CLTF1);
Outer_stepinfo2 = stepinfo(Outer_CLTF2);
Outer_stepinfo3 = stepinfo(Outer_CLTF3);
display(Outer_stepinfo1);


%% Simulate simulink model
sim('discreteOuter',t_sim);


%% Time Delay
% The following was used to determine the time delay of 0.011 seconds:
dclk = squeeze(d_clock.Data);
figure
plot(d_clock.time,dclk)
title('Time Delay')
xlabel('sample')
ylabel('delay (seconds)')


%% Compare Results
% These plots show the performance differences between the simulation and
% the SIL implementations of the outer loop controller.
figure
plot(T_B_sim)
hold on
plot(T_B_sil)
legend('SIM 1', 'SIM 2', 'SIM 3', 'SIL 1', 'SIL 2', 'SIL 3')
title('Commanded Torque')

figure
plot(wbi_B_sim)
hold on
plot(wbi_B_sil)
legend('SIM 1', 'SIM 2', 'SIM 3', 'SIL 1', 'SIL 2', 'SIL 3')
title('Angular Velocity')

figure
plot(theta_sim)
hold on
plot(theta_sil)
legend('SIM 1', 'SIM 2', 'SIM 3', 'SIL 1', 'SIL 2', 'SIL 3')
title('Angle Error')
