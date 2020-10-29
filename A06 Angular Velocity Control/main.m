%% Angular Velocity Control Assignment
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

figure
bode(C1*G1, C2*G2, C3*G3, {1,1000}, '--');
title('Open Loop Bode Plot');
legend('1', '2', '3');

%% 5. Display the phase margin and gain margin of each of the three controlled axes
[GM1, PM1] = margin(C1*G1);
display(mag2db(GM1), 'Gain Margin 1 (dB)');
display(PM1, 'Phase Margin 1 (degrees)');
[GM2, PM2] = margin(C2*G2);
display(mag2db(GM2), 'Gain Margin 2 (dB)');
display(PM2, 'Phase Margin 2 (degrees)');
[GM3, PM3] = margin(C3*G3);
display(mag2db(GM3), 'Gain Margin 3 (dB)');
display(PM3, 'Phase Margin 3 (degrees)');

%% 6. How much additional time delay can be added to your system until it is marginally stable?
dt_unstable = [PM1*pi/180/w_crossover; ...
               PM2*pi/180/w_crossover; ...
               PM3*pi/180/w_crossover];
display(min(dt_unstable), 'Additional Delay to Marginally Stable (s)')
% [GM_unstable, PM_unstable] = margin(C1*G1*exp(-s*dt_unstable(1)));

%% 7. If your estimate of the inertia is wrong, how small does it need to be to drive your system to marginal stability?
J_unstable = [J_C_P(1,1)/GM1; ...
              J_C_P(2,2)/GM2; ...
              J_C_P(3,3)/GM3;];
display(J_unstable, 'Inertial Value to Marginally Stable (kg*m^2)')
% [GM_unstable, PM_unstable] = margin(Kd1*C1*1/(J_unstable(1)*s));
          
%% 8. Plot a Closed Loop Bode Plot for each of the three axes
CLTF1 = feedback(C1*G1, 1);
figure
bode(CLTF1, {1,1000})
title('Closed Loop Bode Plot 1');
CLTF2 = feedback(C2*G2, 1);
figure
bode(CLTF2, {1,1000})
title('Closed Loop Bode Plot 2');
CLTF3 = feedback(C3*G3, 1);
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
display(stepinfo1.Overshoot, 'Overshoot 1 (%)');
display(stepinfo2.Overshoot, 'Overshoot 2 (%)');
display(stepinfo3.Overshoot, 'Overshoot 3 (%)');

%% 12. Plot the results of your simulink simulation.  
%      Does the simscape model behave like the three linear sisomodels?
% The Simscape model does behave very much like the linear systems. 
% Only in the transient does it vary on the order of mrads/s.

% Run the Simulation
sim('VelocityControl',t_sim)

% Plot the simulation result
figure
plot(wbi_B)
title('Angular Velocity Simscape Simulation');
legend('\omega_B_I_x','\omega_B_I_y','\omega_B_I_z','Interpreter','tex')
xlabel('seconds');
ylabel('radians/s');

% Plot the linear systems result
figure
plot(wXYZ_B)
title('Angular Velocity Linear Systems');
legend('\omega_X','\omega_Y','\omega_Z','Interpreter','tex')
xlabel('seconds');
ylabel('radians/s');

% Plot the difference
figure
plot(wXYZ_B-wbi_B)
title('Angular Velocity Difference');
legend('\omega_X','\omega_Y','\omega_Z','Interpreter','tex')
xlabel('seconds');
ylabel('radians/s');

%% 13. Comment on your results
% The controller that was developed was simple to implement. By a small 
% amount of trial and error an appropriate Phase Margin was chosen such
% that all other design criteria was met. Also the closed loop Bode plots
% and the simulation results show that there is not a long settling period
% or great amount of oscillating resonance in the closed loop system.
% The difference in the two simulations is due to the feed forward
% linearization, all other things being equal. It builds confidence to see
% that the feed forward torque method for linearization and decoupling
% works so well.
