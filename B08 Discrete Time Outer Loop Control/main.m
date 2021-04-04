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
t_sim = 2; % seconds
dt_delay = 0.011; % seconds
dt_sample = 0.01; % seconds

% initial angular velocity
wbi0_B = [0;0;0];

% Initial attitude
q0_BI.s = 1;
q0_BI.v = [0;0;0];

% Desired attitude
qstar_BI.s = 1;
qstar_BI.v = [0;0;0];


%% Plant models

% Define transfer function variables
s = tf('s');
z = tf('z');

G1 = 1/(J_C_P(1,1)*s);
G2 = 1/(J_C_P(2,2)*s);
G3 = 1/(J_C_P(3,3)*s);

% convert to discrete time
G1d = c2d(G1, dt_delay, 'zoh');
G2d = c2d(G2, dt_delay, 'zoh');
G3d = c2d(G3, dt_delay, 'zoh');


%% Control design

% redesign using 0.011 second delay
% controlSystemDesigner(G1d/z);

% chosen for 60deg phase margin and equal response for each axis:
K1d = 0.2235; % C1 = 0.2235 gives PM 60deg at 5.04Hz, GM 9.2dB at 15.2Hz
K2d = 1.214; % C2 = 1.214 gives PM 60deg at 5.04Hz, GM 9.2dB at 15.2Hz
K3d = 1.2589; % C3 = 1.2589 gives PM 60deg at 5.04Hz, GM 9.2dB at 15.2Hz

% Control gains as diagonal matrix for input into simulink.
Kd = diag([K1d; K2d; K3d]);

%% Open Loop Analysis
% The bode plots show as near to matching response as is feasible. The gain
% and phase margin calculations support this finding as well.
figure
bode(K1d*G1d/z);
hold on
bode(K2d*G2d/z);
bode(K3d*G3d/z);
title('Open Loop Bode Plot');
legend('1', '2', '3');

[GM1, PM1] = margin(K1d*G1d/z);
display(mag2db(GM1), 'Gain Margin 1 (dB)');
display(PM1, 'Phase Margin 1 (degrees)');
[GM2, PM2] = margin(K2d*G2d/z);
display(mag2db(GM2), 'Gain Margin 2 (dB)');
display(PM2, 'Phase Margin 2 (degrees)');
[GM3, PM3] = margin(K3d*G3d/z);
display(mag2db(GM3), 'Gain Margin 3 (dB)');
display(PM3, 'Phase Margin 3 (degrees)');

%% Closed Loop Analysis

% closed loop transfer function bode plots
CLTF1d = feedback(K1d*G1d/z, 1);
CLTF2d = feedback(K2d*G2d/z, 1);
CLTF3d = feedback(K3d*G3d/z, 1);
figure
bode(CLTF1d, {1,1000})
hold on
bode(CLTF2d, {1,1000})
bode(CLTF3d, {1,1000})
title('Closed Loop Bode Plot');
legend('1', '2', '3');

% closed loop transfer function bandwidth calculation
display(bandwidth(CLTF1d)/(2*pi), 'Closed Loop 3dB Bandwidth 1 (Hz)');
display(bandwidth(CLTF2d)/(2*pi), 'Closed Loop 3dB Bandwidth 2 (Hz)');
display(bandwidth(CLTF3d)/(2*pi), 'Closed Loop 3dB Bandwidth 3 (Hz)');

%% Analytical Step Responses
figure
step(CLTF1d)
hold on
step(CLTF2d)
step(CLTF3d)
legend('1', '2', '3', 'Location', 'southeast');
stepinfo1 = stepinfo(CLTF1d);
display(stepinfo1, 'Step Info 1');
stepinfo2 = stepinfo(CLTF2d);
display(stepinfo2, 'Step Info 2');
stepinfo3 = stepinfo(CLTF3d);
display(stepinfo3, 'Step Info 3');

%% Simulate
% sim('discreteInner',t_sim) % ran previous to generate output

%% Time Delay
% The following was used to determine the time delay of 0.011 seconds:
lbclk = [];
for i = 1:size(lb_clock.Time,1)
    lbclk = [lbclk; lb_clock.Data(:,:,i)];
end
clk = clock.Data(1:size(lbclk,1));
dclock = clk - lbclk;
figure
plot(dclock)
title('Time Delay')
xlim([1 2000])
xlabel('sample')
ylabel('delay (seconds)')


%% Comparison Results
figure
plot(wbi_B_SIL)
hold on
plot(wbi_B_SIM)
plot(wbistar_B)
title('wbi')
legend('SIL 1','SIL 2','SIL 3','SIM 1','SIM 2','SIM 3','* 1','* 2','* 3','Location','southeast')
xlim([1.99 2.2])

