%% Reaction Wheel Control
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

%%
% Reaction Wheel Properties
wn = 2*pi*10; % Reaction Wheel Natural Frequency
zeta = sqrt(2)/2; % Reaction Wheel Damping Ratio

%%
% Initial Reaction Wheel Angular Momentum
hw0_B = [0;0;0];

%% Parameters
dt_delay = 0.01; % seconds

% amount of time to run simulation
t_sim = 2.0; % seconds

% Initial attitude
e = [0; 1; 0]; e = e/norm(e);
q0_BI = e2q(e, -90*pi/180);
% Desired Attitude
qstar_BI = e2q(e, 90*pi/180);
ramp_slope = pi; % ramp slope (rad/s)
A0_BI = q2A(q0_BI);
A0_IB = A0_BI';

% Initial Satellite Angular Velocity
wbi0_B = [0; 0; 0];%*pi/180; % rad/s
wbi0_P = A_PB*wbi0_B;

% Define transfer function variable
s = tf('s');

% Disturbance parameters
d_type = -1; % 1 for constant, 3 for sine, else zero
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
% The Reaction Wheel system bode plots show that they act like a low pass
% filter. The step response looks close to a 65 degree phase margin, with
% some overshoot, but a smooth response without resonance.

% Reaction Wheel Transfer Function
Gw = wn^2/(s^2 + 2*zeta*wn*s + wn^2);

figure
bode(Gw);
title('Reaction Wheel Closed Loop Bode Plot');

figure
step(Gw);
title('Reaction Wheel Step Response');

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
display(Kd1);
Kd2 = 1/bode(G2*Gw, w_crossover);
display(Kd2);
Kd3 = 1/bode(G3*Gw, w_crossover);
display(Kd3);

% Set control gains as diagonal matrix for input into simulink.
Kd = diag([Kd1; Kd2; Kd3]);

%% Open Inner Loop Bode Plot of each axis with the controller in the loop
% The open inner loop bode plot looks about the same as without the
% reaction wheels.

% Define each controller
C1 = tf(Kd1*C_pade8);
display(tf(Kd1,'OutputDelay', dt_delay));
C2 = tf(Kd2*C_pade8);
display(tf(Kd2,'OutputDelay', dt_delay));
C3 = tf(Kd3*C_pade8);
display(tf(Kd3,'OutputDelay', dt_delay));

figure
bode(C1*G1*Gw, C2*G2*Gw, C3*G3*Gw, {1,1000}, '--');
title('Open Inner Loop Bode Plot');
legend('1', '2', '3');


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
figure
bode(Inner_CLTF1, Inner_CLTF2, Inner_CLTF3, {1,1000}, '--');
legend('1', '2', '3');
title('Closed Inner Loop Bode Plots');

%% Closed Inner Loop Bandwidth
% Bandwidth is exactly the same for each axis.
display(bandwidth(Inner_CLTF1)/(2*pi), 'Inner Loop bandwidth 1 (Hz)');


%% Closed Inner Loop Step responses
% The step response plot of the closed inner loop looks smooth without any
% noticable resonance and a good rise and settling. There is not too much
% overshoot either.
figure
step(Inner_CLTF1, Inner_CLTF2, Inner_CLTF3, 0.3, '--');
legend('1', '2', '3');
title('Closed Inner Loop Step Responses');


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

Ko = 3075; % outer loop control gain
Zo = -0.01; % zero location (rad/s)
Po = -400; % pole location (rad/s)
Co = Ko*(s-Zo)/(s*(s-Po));
config.C1.value = Co;
% controlSystemDesigner(config);


%% Open Outer Loop Bode Plot
% The outer open loop bode plot shows the effect of the slow zero, it has a
% large region of phase above 180 degrees. The pole causes the high
% frequency to have a greater negative slope in magnitude.
Outer_OLTF1 = Co*Inner_CLTF1*Go;
Outer_OLTF2 = Co*Inner_CLTF2*Go;
Outer_OLTF3 = Co*Inner_CLTF3*Go;
figure
bode(Outer_OLTF1, Outer_OLTF2, Outer_OLTF3, {0.001, 1000}, '--');
legend('1', '2', '3');
title('Open Outer Loop Bode Plot');

%% Closed Outer Loop Bode Plot
% Closed outer loop transfer functions show a magnitude response above the
% bandwidth with a steep negative slope. The phase response is more
% significantly affected at frequencies lower than that.
Outer_CLTF1 = feedback(Co*Inner_CLTF1*Go,1);
Outer_CLTF2 = feedback(Co*Inner_CLTF2*Go,1);
Outer_CLTF3 = feedback(Co*Inner_CLTF3*Go,1);
figure
bode(Outer_CLTF1, Outer_CLTF2, Outer_CLTF3, {0.01, 1000}, '--');
legend('1', '2', '3');
title('Closed Outer Loop Bode Plot');


%% Relative Stability Margins
% The phase margin and gain margin of the outer loop indicate that this
% system is reliably stable.
[GMO1, PMO1] = margin(Outer_OLTF1);
display(mag2db(GMO1), 'Gain Margin 1 (dB)');
display(PMO1, 'Phase Margin 1 (degrees)');


%% Step Response Performance Characteristics
% Outer loop step response performance characteristics show a settling time
% of 0.4 seconds. This is fine. The tradeoffs would either cause less
% stability or less responsive disturbance rejection. The overshoot is
% about 7%, which seems high, but this was chosen because timing was higher
% priority.
Outer_stepinfo1 = stepinfo(Outer_CLTF1);
display(Outer_stepinfo1);


%% Disturbance Transfer Function Bode Plot
% As seen in the disturbance transfer function bode plots there is a large
% band of amplification for one axis, but careful inspection of the units
% assures that unless a large disturbance of around 10Nm is in effect the
% system should attenuate it even at those frequencies. The disturbance
% transfer function is a bandpass filter, so given time the contant or high
% frequency disturbances will completely disappear. The phase response of
% disturbance starts with +90 degree, falling to zero in the middle band,
% dropping another 90 degrees after. 
DTF1 = G1*Go/(1 + C1*G1 + C1*Co*G1*Go);
DTF2 = G2*Go/(1 + C2*G2 + C2*Co*G2*Go);
DTF3 = G3*Go/(1 + C3*G3 + C3*Co*G3*Go);
figure
bode(DTF1, DTF2, DTF3, {.0001, 10000}, '--');
legend('1', '2', '3');
title('Disturbance Transfer Functions Bode Plot');


%% Step Response of the Disturbance Transfer Function
% This system rejects constant disturbance over time after the controller
% is turned on. By the time 10 minutes have passed, the effect is
% negligible.
figure
step(DTF1, DTF2, DTF3, 700, '--');
legend('1', '2', '3');
title('Step Response of the Disturbance Transfer Function');


%% Simulink Simulation - Step Response
% The simulation matches the response as shown in the analysis above with
% regard to the behavior of the full system given a step input.
input_type = 1; % constant input
input_param = 0; % unused parameter for constant input type
sim('AngleControlWheels', t_sim)
figure
plot(theta_in, ':')
hold on
plot(theta_out)
title('Simulation Step Response')


%% Simulink Simulation - Ramp Response
% As seen in these plots, the ramp response error disappears after about 10
% minutes.
input_type = 2; % ramp input
q0_BI = e2q(e, 0*pi/180);
A0_BI = q2A(q0_BI);
A0_IB = A0_BI';
qstar_BI = q0_BI;
ramp_slope = pi; % ramp slope (rad/s)
input_param = ramp_slope;
t_sim = 700.0;
sim('AngleControlWheels', t_sim)
figure
plot(theta_in.Time(1:12000), theta_in.Data(1:12000,:), ':')
hold on
plot(theta_out.Time(1:12000), theta_out.Data(1:12000,:))
title('Simulation Ramp Response')
figure
plot(theta_error)
title('Simulation Ramp Response Error')


%% Simulink Simulation - Sinusoidal Input
% The attenuation amount is exactly what is expected when looking at the
% sine output after the transients. The phase response accounts for most of
% the error signal given the sinusiodal input.
input_type = 3; % sinusoidal input
freq = bandwidth(Outer_CLTF1); % frequency of 3dB bandwidth (rad/s)
display(freq*180/pi, '3dB bandwidth frequency (Hz)');
input_param = freq;
t_sim = 6;
sim('AngleControlWheels', t_sim)
figure
plot(theta_in, ':')
hold on
plot(theta_out)
title('Simulation Sinusiodal Response')
figure
plot(theta_error)
title('Simulation Sinusoidal Response Error')


%% Simulink Simulation - Constant Disturbance
% 
e = [3; 5; 2]; e = e/norm(e);
q0_BI = e2q(e, 0*pi/180);
A0_BI = q2A(q0_BI);
A0_IB = A0_BI';
qstar_BI = q0_BI;
input_type = 1; % constant input
d_type = 1; % constant disturbance
dist_level = 0.0001;
t_sim = 700;
sim('AngleControlWheels', t_sim)
figure
plot(theta_error)
title('Simulation of Constant Disturbance')


%% Simulink Simulation - Sinusoidal Disturbance
% 
q0_BI = e2q(e, 0*pi/180);
qstar_BI = q0_BI;
A0_BI = q2A(q0_BI);
A0_IB = A0_BI';
input_type = 1; % constant input
d_type = 3; % sinusoidal disturbance
freq = bandwidth(Outer_CLTF1); % frequency of 3dB bandwidth (rad/s)
input_param = freq;
t_sim = 6;
sim('AngleControlWheels', t_sim)
figure
plot(theta_out)
title('Simulation of Sinusiodal Disturbance')


%% Reaction Wheel Attitude Control Summary
% Adding the reaction wheel transfer functions to the model did little to
% change the overall behavior of the system. The settling time was and
% other timing performance criteria were affected the most.

%% Conclusions
% Adding the reaction wheels to the model was quite simple. Doing so brings
% it one step closer to being a much more realistic system. This step
% should not be avoided because there is no way to instantaneously apply
% torque to any satellite and any useful model should reflect this reality.
