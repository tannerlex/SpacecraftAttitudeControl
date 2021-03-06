%% Momentum Dumping - Assignment 3
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

% Load dimensions
dimensions

%% Parameters
dt_delay = 0.01; % seconds
Mmax = 0.17; % Maximum magnetic moment (A-m^2)
tau = 1/(100*2*pi); % time constant of the torque coils

%%
% Reaction Wheel Properties
wn = 2*pi*10; % Reaction Wheel Natural Frequency
zeta = sqrt(2)/2; % Reaction Wheel Damping Ratio
hwmax = 0.015; % Nms
hwdotmax = 0.004; % Nm
safety = 0.5; % reaction wheel safety factor
wmax = safety*hwmax/Jmax;
wdotmax = safety*hwdotmax/Jmax;

% amount of time to run simulation
t_sim = 3600; % seconds

% Initial conditions
wbi0_B = [0;0;0]; % Initial Satellite Angular Velocity rad/s
hw0_B = [-5;5;5]*1e-3; % Initial Reaction Wheel Angular Momentum Nms
% [1;0;0;0]; % Initial orientation of the B frame relative to the I frame.
q0_BI.s = 1;
q0_BI.v = [0;0;0];
A0_BI = q2A(q0_BI);
A0_IB = A0_BI';

%% Desired
hwstar_B = [0;0;0];
qstar_BI = q0_BI;

%%
% Define transfer function variable
s = tf('s');

% Calculate the principal open loop plant models
G1 = 1/(J_C_P(1,1)*s);
display(G1);
G2 = 1/(J_C_P(2,2)*s);
display(G2);
G3 = 1/(J_C_P(3,3)*s);
display(G3);


%% Reaction Wheels
% Reaction Wheel Transfer Function
Gw = wn^2/(s^2 + 2*zeta*wn*s + wn^2);


%% Inner Loop Proportional Control Design
% The design for the inner loop should ensure all axes have identical
% response. I used controlSystemDesigner to design a controller with a
% phase margin of ~60 degrees. This gives a crossover frequency of 2.46 Hz.

% Use 8th order Pade approximation for the time delay of the system.
[num, den] = pade(dt_delay, 8);
C_pade8 = tf(num, den);

config = sisoinit(1);
config.C.value = 1*Gw*C_pade8; % initially just 1, to be tuned below
config.G.value = G1;
config.OL1.View = {'bode'};
% controlSystemDesigner(config);

% using the crossover frequency found by adjusting the gain in
% controlSystemDesigner to achieve ~60deg PM. Actual PM: 60.8, freq: 2.46
w_crossover = 2*pi*2.46; % rad/s
display(w_crossover/(2*pi), 'Open Loop Gain Crossover Frequency (Hz)');

% Proportional control gains ensure all three axes have identical response.
Kd1 = 1/bode(G1*Gw, w_crossover);
% display(Kd1);
Kd2 = 1/bode(G2*Gw, w_crossover);
% display(Kd2);
Kd3 = 1/bode(G3*Gw, w_crossover);
% display(Kd3);

% Set control gains as diagonal matrix for input into simulink.
Kd = diag([Kd1; Kd2; Kd3]);

%% Open Inner Loop Bode Plot of each axis with the controller in the loop
% The open inner loop bode plot is exactly the same for each axis.

% Define each controller
C1 = tf(Kd1*C_pade8);
display(tf(Kd1,'OutputDelay', dt_delay));
C2 = tf(Kd2*C_pade8);
display(tf(Kd2,'OutputDelay', dt_delay));
C3 = tf(Kd3*C_pade8);
display(tf(Kd3,'OutputDelay', dt_delay));

% figure
% bode(C1*G1*Gw, C2*G2*Gw, C3*G3*Gw, {1,1000}, '--');
% title('Open Inner Loop Bode Plot');
% legend('1', '2', '3');


%% Inner Loop Phase and Gain Margins
% The gain and phase margins are exactly the same for each axis.
[GM1, PM1] = margin(C1*G1*Gw);
display(mag2db(GM1), 'Gain Margin 1 (dB)');
display(PM1, 'Phase Margin 1 (degrees)');


%% Closed Inner Loop Bode Plot for each of the three axes
% As expected the inner loop, once closed with feedback, looks like a low
% pass filter.
Inner_CLTF1 = feedback(C1*G1*Gw, 1);
Inner_CLTF2 = feedback(C2*G2*Gw, 1);
Inner_CLTF3 = feedback(C3*G3*Gw, 1);
% figure
% bode(Inner_CLTF1, Inner_CLTF2, Inner_CLTF3, {1,1000}, '--');
% legend('1', '2', '3');
% title('Closed Inner Loop Bode Plots');


%% Closed Inner Loop Bandwidth
% Bandwidth is exactly the same for each axis.
display(bandwidth(Inner_CLTF1)/(2*pi), 'Inner Loop bandwidth 1 (Hz)');


%% Inner Loop Step Response Information
% The step responses are all exactly the same across the three axes. The
% settling time is a nice 0.2 seconds. Having reaction wheels in the model
% have slowed this down significantly, but it is much closer to reality.
stepinfo1 = stepinfo(Inner_CLTF1);
display(stepinfo1);


%% Outer Loop Control Design
% The outer loop control uses a lead to buy some phase and create a range
% of stability, but cuts out the high frequency response of the system.
Go = 1/s;
config = sisoinit(6);
config.G1.value = G1;
config.C1.value = 1; % initially just 1, to be tuned below
config.C2.value = Kd1*C_pade8*Gw;
config.G2.value = Go;
config.OL1.View = {'bode'};
config.OL2.View = {};
% controlSystemDesigner(config);

% Design proportional controller
Kp = 8.0;

% Design with lead
Ko = 3075; % outer loop control gain
Zo = 0.01; % zero location (rad/s)
Po = 400; % pole location (rad/s)
Co = Ko*(s+Zo)/(s*(s+Po));
config.C1.value = Co;
% controlSystemDesigner(config);


%% Open Outer Loop Transfer Function
Outer_OLTF1 = Co*Inner_CLTF1*Go;
Outer_OLTF2 = Co*Inner_CLTF2*Go;
Outer_OLTF3 = Co*Inner_CLTF3*Go;

%% Relative Stability Margins
% The phase margin and gain margin of the outer loop meet the requirements:
% PM >= 60deg, GM >= 6dB
[GMO1, PMO1] = margin(Outer_OLTF1);
display(mag2db(GMO1), 'Gain Margin 1 (dB)');
display(PMO1, 'Phase Margin 1 (degrees)');


%%
% Momentum Dumping Transfer Function
Gm = 1/(tau*s+1) * 1/s * C_pade8;
Km = 1/bode(Gm, 2*pi*0.00034); % rad/sec
display(Km, 'Proportional Gain for momentum dumping control');

figure
margin(Km*Gm);

figure
[Y,T] = step(feedback(Km*Gm,1));
plot(T,5e-3*(1-Y))
title('Step Error Rejection');

%% Simulation and Results
sim('MomentumDumping', 3600);

figure
plot(theta_error)
title('Error Angle (degrees)')

figure
plot(Mstar_B)
title('Commanded Magnetic Moment (A-m^2)')

figure
plot(hw_B)
title('Reaction Wheel Angular Momentum (Nms)')
