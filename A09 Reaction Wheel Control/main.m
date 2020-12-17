%% Reaction Wheel Control - Assignment 9
% Name: Tanner Lex Jones

%% Preliminaries

% This cleans all variables and sets the format to display more digits.
clearvars
close all
clc
format long
% warning('off')

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
Jmax = max(max(abs(J_C_P)));

%% Parameters
dt_delay = 0.01; % seconds

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
t_sim = 20; % seconds

% Initial conditions
wbi0_B = [0;0;0]; % Initial Satellite Angular Velocity rad/s
hw0_B = [0;0;0]; % Initial Reaction Wheel Angular Momentum Nms
% [1;0;0;0]; % Initial orientation of the B frame relative to the I frame.
q0_BI.s = 1;
q0_BI.v = [0;0;0];

% Desired Attitude
e = [1; 2; 3]; e = e/norm(e);
qstar_BI = e2q(e, 180*pi/180);
ramp_slope = pi; % ramp slope (rad/s)
A0_BI = q2A(q0_BI);
A0_IB = A0_BI';

input_type = 1; % constant input
input_param = 0; % unused parameter for constant input type

wbi0_P = A_PB*wbi0_B;

% Define transfer function variable
s = tf('s');

% Disturbance parameters
d_type = 0; % 1 for constant, 3 for sine, else zero
dist_level = 0.0001;

%%
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


%% Simulink Simulation - 1. Step Response using Integrator/Lead Controller
% The simulation can enter a limit cycle depending on the given step input.
% Because we need stability as the first priority this shows that the
% design is not complete.
% 
% *How long did it take to achieve the final desired attitude?*
% It takes over 100 seconds in this case.
% 
% *Discuss the attitude error during the slew maneuver.*
% The error shows huge overshoots that it keeps trying to compensate for,
% but it eventually settles in this example.
% 
% *Discuss the reaction wheel angular momentum and torque during the slew
% maneuver.*
% The reaction wheels limit out on torque in one direction or the other the
% whole time causing the very triangular wave shaped angular momentum and
% the square wave shaped torque.

% ensure that the reset will not activate
safety = 1.1;

t_sim = 200;
sim('AngleControlWheels', t_sim)
figure
plot(theta_error);
title('1. Step Command Using Integrator/Lead - Angle Error')
figure
plot(hw_B);
title('1. Step Command Using Integrator/Lead - Wheel Angular Momentum');
figure
plot(hwdot_B);
title('1. Step Command Using Integrator/Lead - Wheel Angular Torque');


%% Simulink Simulation - 2. Step Response using proportional control switch
% When a proportional control is added on the condition that the reaction
% wheels are within the safety factor of saturation, stability is acheived,
% but the performance is poor.
% 
% *How long did it take to achieve the final desired attitude?*
% 100 seconds, only slightly better than the previous example when it comes
% to performance for this case.
% 
% *Discuss the attitude error during the slew maneuver.*
% It is really reactive and seems resonant on a certain frequency, but
% eventually settles down. Ultimately it is stable, but can't be considered
% as performing the command well.
% 
% *Discuss the reaction wheel angular momentum and torque during the slew
% maneuver.*
% The wheels hit the safety limits a lot while trying to stabilize and
% reach the commanded attitude. This causes a lot of overshoot which is at
% the heart of the poor performance.
% It is interesting to note that the axis with the largest moment of
% inertia takes longer to stabilize than the other two. This is because all
% the reaction wheels were given the same limits.

% engage the reset option
safety = 0.5;

t_sim = 150;
sim('AngleControlWheels', t_sim)
figure
plot(theta_error);
title('2. Step Command w/ Windup Fix - Angle Error')
figure
plot(hw_B);
title('2. Step Command w/ Windup Fix - Wheel Angular Momentum');
figure
plot(hwdot_B);
title('2. Step Command w/ Windup Fix - Wheel Angular Torque');


%% Simulink Simulation - 3. Profiled trapezoidal trajectory
% By only giving the system a command that it can actually fulfill the
% controller tracks the desired attitude really well. It always lags, but
% the error is on the order of 10^-3, which is really good.
% 
% *How long did it take to achieve the final desired attitude?*
% 21 seconds, which is remarkable given that the tactic essentially just
% imposes constraints on the input. This is better by ~80 seconds than the
% last example.
% 
% *Discuss the attitude error during the slew maneuver.*
% The error gets almost as high as 20mrad which is great performance for
% tracking the trajectory slew. It also decreases over time. Most of the
% error accounts for the lag behind the commanded input and the ability of
% the controller to catch up.
% 
% *Discuss the reaction wheel angular momentum and torque during the slew
% maneuver.*
% The angular momentum looks like really good trapezoids as expected. In
% the torque you can see that small corrections happen along the way with
% some higher frequency bumps.

t_sim = 30;
sim('SatelliteAttitude', t_sim)
figure
plot(theta_error);
title('3. Profiled Command - Angle Error')
figure
plot(hw_B);
title('3. Profiled Command - Wheel Angular Momentum');
figure
plot(hwdot_B);
title('3. Profiled Command - Wheel Angular Torque');


%% Simulink Simulation - 4. Command Feedforward
% Using the command feedforward modification the error during the slew is
% very small. For the most part it is within numerical noise. The only
% noticable errors happen at the transitions between turning the torque on
% and off.
% 
% *How long did it take to achieve the final desired attitude?*
% 21 seconds, same as the last example.
% 
% *Discuss the attitude error during the slew maneuver.*
% The error here shows how effective this system is at tracking a good
% profiled input. Other than the transition regions where the torque makes
% a big change the only error is in the numerical noise. Those transition
% regions show error on the order of less than 10^-4!
% 
% *Discuss the reaction wheel angular momentum and torque during the slew
% maneuver.*
% They look as good as possible with the outer loop parameters I gave the
% system. The overshoot that you get with 60deg PM shows. The edges of the
% torque changes show that behavior.

sim('SatelliteAttitudeControl', t_sim)
figure
plot(theta_error);
title('4. Feedforward w/ Trapezoidal Command - Angle Error')
figure
plot(hw_B);
title('4. Feedforward w/ Trapezoidal Command - Wheel Angular Momentum');
figure
plot(hwdot_B);
title('4. Feedforward w/ Trapezoidal Command - Wheel Angular Torque');

