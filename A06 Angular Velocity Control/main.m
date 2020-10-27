%% Angular Velocity Control Assignment
% Name: Tanner Lex Jones

%% Preliminaries
% This cleans all variables and sets the format to display more digits.
clearvars
close all
clc
format long

%%
ctrlpref

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

A0_BI = q2A(q0_BI);
A0_IB = A0_BI';

% Initial Satellite Angular Velocity
wbi0_B = [5; -10; 15]*pi/180; % rad/s
wbi0_P = A_PB*wbi0_B;

% Define transfer function variable
s = tf('s');

%% 1. What are each of the three principal open loop plants?
% Plant model
G1 = 1/(J_C_P(1,1)*s);
display(G1);
G2 = 1/(J_C_P(2,2)*s);
display(G2);
G3 = 1/(J_C_P(3,3)*s);
display(G3);

%% 2. What is the open loop gain crossover frequency in the controller?
PM = 60*pi/180; % Phase Margin (rad)
DC_phase = -90*pi/180; % Phase at DC (rad)
% -pi = DC_phase - Phase Margin - Xover_angle
% Xover_angle = DC_phase -PM + pi
Xover_angle = DC_phase - PM + pi;

% crossover frequency
w_crossover = Xover_angle/dt_delay; % rad/s
display(w_crossover, 'Open Loop Gain Crossover Frequency (rad/s)');

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

figure
bode(C1*G1, C2*G2, C3*G3, {1,1000}, '--');
title('Open Loop Bode Plot');
legend('1', '2', '3');

%% 5. Display the phase margin and gain margin of each of the three controlled axes
[GM1, PM1] = margin(Kd1*C1*G1);
display(mag2db(GM1), 'Gain Margin 1 (dB)');
display(PM1, 'Phase Margin 1 (degrees)');
[GM2, PM2] = margin(Kd2*C2*G2);
display(mag2db(GM2), 'Gain Margin 2 (dB)');
display(PM2, 'Phase Margin 2 (degrees)');
[GM3, PM3] = margin(Kd3*C3*G3);
display(mag2db(GM3), 'Gain Margin 3 (dB)');
display(PM3, 'Phase Margin 3 (degrees)');

%% 6. How much additional time delay can be added to your system until it is marginally stable?
dt_unstable = [PM1*pi/180/w_crossover; ...
               PM2*pi/180/w_crossover; ...
               PM3*pi/180/w_crossover];
display(min(dt_unstable), 'Additional Delay to Marginally Stable (s)')
% [GM_unstable, PM_unstable] = margin(Kd3*C3*G3*exp(-s*dt_unstable(3)));% huh?

%% 7. If your estimate of the inertia is wrong, how small does it need to be to drive your system to marginal stability?
J_unstable = [J_C_P(1,1)/GM1; ...
              J_C_P(2,2)/GM2; ...
              J_C_P(3,3)/GM3;];
display(min(J_unstable), 'Inertial Value to Marginally Stable (kg/m)') % units?
% [GM_unstable, PM_unstable] = margin(Kd1*C1*1/(J_unstable(1)*s));
          
%% 8. Plot a Closed Loop Bode Plot for each of the three axes
CLTF1 = feedback(Kd1*C1*G1, 1);
figure
bode(CLTF1, {1,1000})
title('Closed Loop Bode Plot 1');
CLTF2 = feedback(Kd2*C2*G2, 1);
figure
bode(CLTF2, {1,1000})
title('Closed Loop Bode Plot 2');
CLTF3 = feedback(Kd3*C3*G3, 1);
figure
bode(CLTF3, {1,1000})
title('Closed Loop Bode Plot 3');

%% 9. What is the closed loop 3dB bandwidth for each of the three axes?
display(bandwidth(CLTF1)/(2*pi), 'Closed Loop 3dB Bandwidth 1 (Hz)');
display(bandwidth(CLTF2)/(2*pi), 'Closed Loop 3dB Bandwidth 2 (Hz)');
display(bandwidth(CLTF3)/(2*pi), 'Closed Loop 3dB Bandwidth 3 (Hz)');

%% 10. What is the rise time for each of the three axes?
stepinfo1 = stepinfo(CLTF1);
display(stepinfo1.RiseTime, 'Rise Time 1 (s)');
stepinfo2 = stepinfo(CLTF2);
display(stepinfo2.RiseTime, 'Rise Time 2 (s)');
stepinfo3 = stepinfo(CLTF3);
display(stepinfo3.RiseTime, 'Rise Time 3 (s)');

%% 11. What is the overshoot for each of the three axes?
display(stepinfo1.Overshoot, 'Overshoot 1 '); % units?
display(stepinfo2.Overshoot, 'Overshoot 2 ');
display(stepinfo3.Overshoot, 'Overshoot 3 ');

%% 12. Plot the results of your simulink simulation.  
%      Does the simscape model behave like the three linear sisomodels?


%% 13. Comment on your results

