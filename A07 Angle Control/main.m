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
t_sim = 6; % seconds

% Initial attitude
e = [1; 1; 1]; e = e/norm(e);
q0_BI = e2q(e, 45*pi/180);
% Desired Attitude
qstar_BI = e2q(e, 225*pi/180);

A0_BI = q2A(q0_BI);
A0_IB = A0_BI';

% Define transfer function variable
s = tf('s');

%%
% Calculate the principal open loop plant models
G1 = 1/(J_C_P(1,1)*s);
display(G1);
G2 = 1/(J_C_P(2,2)*s);
display(G2);
G3 = 1/(J_C_P(3,3)*s);
display(G3);

%% Inner Loop Proportional Control Design

% Open loop gain crossover frequency in the controller
PM = 65*pi/180; % Phase Margin (rad)
DC_phase = -90*pi/180; % Phase at DC (rad)
% -pi = DC_phase - Phase Margin - Xover_angle
% Xover_angle = DC_phase -PM + pi
Xover_angle = DC_phase - PM + pi;

% crossover frequency
w_crossover = Xover_angle/dt_delay; % rad/s
display(w_crossover/(2*pi), 'Open Loop Gain Crossover Frequency (Hz)');

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

%% Open Inner Loop Bode Plot of each axis with the controller in the loop
% This bode plot shows the response of the open loop system.
% The magnitude shows a response of logarithmic decrease and a phase that 
% starts at -90deg and rolling off which are both typical of a proportional
% controller with an integrator plant. The three axes are equal because the
% gains were all calculated to achieve the same crossover frequency.

% Define each controller
C1 = tf(Kd1,'OutputDelay', dt_delay);
display(C1);
C2 = tf(Kd2,'OutputDelay', dt_delay);
display(C2);
C3 = tf(Kd3,'OutputDelay', dt_delay);
display(C3);

figure
bode(C1*G1, C2*G2, C3*G3, {1,1000}, '--');
title('Open Inner Loop Bode Plot');
legend('1', '2', '3');


%% Closed Inner Loop Bode Plot for each of the three axes
% This closed loop bode of the inner loop show that this part of the system
% acts as a low pass filter which is seen by the magnitude plot. It is
% mostly flat in the lower frequencies and after the 3db bandwidth
% frequency (~14Hz) it rolls off more steeply. The phase also decreases
% with increase in frequency, starting at 0deg and already at ~-115deg at 
% the bandwidth frequency.
CLTF1 = feedback(C1*G1, 1);
CLTF2 = feedback(C2*G2, 1);
CLTF3 = feedback(C3*G3, 1);
figure
bode(CLTF1, CLTF2, CLTF3, {1,1000})
title('Closed Inner Loop Bode Plots');


%% Closed Inner Loop Step responses
% The step responses also are exactly the same. You can see the rise
% and settling time are both good in this plot. Also notice that the
% overshoot appears to be small.
figure
step(CLTF1, CLTF2, CLTF3, 0.1)
title('Closed Inner Loop Step Responses');


%% Inner Loop Phase and Gain Margins
% Each axis has the same phase and gain margins: 65deg, and 11.126dB. The
% PM is above the design requirement by 5 degrees and the GM is well above
% the requirement of 6dB. These margins will ensure good stability even if
% the true values may be off due to measurements or other errors.
[GM1, PM1] = margin(C1*G1);
display(mag2db(GM1), 'Gain Margin 1 (dB)');
display(PM1, 'Phase Margin 1 (degrees)');


%% Inner Loop Step Response Information
% Because the design has an equal phase margin for each axis the step
% response is also exactly the same. Choosing the 65deg phase margin gives
% a quick 24ms rise time, with a small overshoot of 0.8%. The most 
% important design criteria was first stability and then settling time,
% which was 45ms.
% With a faster system, for example 60deg PM, the settling time increased
% to 60ms, and a slower system (70deg PM) also had increased settling time
% (72ms).
stepinfo1 = stepinfo(CLTF1);
display(stepinfo1);


%% Outer Loop Control Design
% The first design criteria is to stabilize the system. The next most
% important criteria was again chosen to be the settling time of the step
% response. For this the PM was chosen to be 65deg.

[num, den] = pade(dt_delay, 8);
C_pade8 = tf(num, den);

config = sisoinit(6);
config.G1.value = G1;
config.C1.value = 1; % initially just 1, to be tuned below
config.C2.value = Kd1*C_pade8;
config.G2.value = 1/s;
config.OL1.View = {'bode'};
config.OL2.View = {};
% controlSystemDesigner(config);

Ko = 2948.2; % outer loop control gain
Zo = -0.01; % zero location (rad/s)
Po = -188.5; % pole location (rad/s)
Co = Ko*(s-Zo)/(s*(s-Po));
OLTF = Co*CLTF1/s;
% config.C1.value = Co;
% controlSystemDesigner(config);


%% Outer Loop Design Questions
% *What is the gain crossover frequency of your outer loop?* 2.46 Hz
% 
% *Why was that gain crossover frequency chosen?* In order to get a 65 deg
% PM because it gives a great settling time.
%
% *How does this gain crossover frequency compare to the gain crossover 
% frequency of the inner loop?* It is at less than half the frequency of the
% inner loop. That is due to the inner loop acting as a lowpass filter. It
% will necessarily decrease the available bandwidth of the outer loop.
%
% *What is the frequency of the real zero?* 1.59E-3 rad/s
% *What effect does the zero have on disturbance rejection?*
% The lower the frequency of the zero, the longer it takes to reject a
% constant disturbance. Increasing the frequency makes the disturbance
% rejection faster, but may also decrease stability because the zero is
% used to buy back phase from the integrator.
%
% *What effect does the zero have on the closed loop step response?*
% The effect of the zero on the system is first to create stability where
% it otherwise could not exist because the phase response of the system is
% -180 deg at DC. The tradeoff of the location of the zero is that when it
% is at higher frequency it cannot quickly settle to a commanded input, but
% it does reject constant disturbance very quickly. On the other hand, when
% it is placed at a very low frequency it will not noticeably affect the
% step response, but it will take much longer to reject a constant
% disturbance.
%
% *What effect does the zero have on the ability to track a ramp?*
% The zero makes the system stable, but it makes it harder to track a ramp
% because it undoes the effect of the integrator. 
%
% *Why did you place the zero where you did?*
% This is a question of the disturbance rejection vs stability. I placed
% the zero such that with ~12 min of control, any constant disturbance is
% rejected. This assumes that the controller will be used for long periods
% of time and that the initial ~12 min is relatively a low cost. Because it
% is a slow zero the system has a lot more range of stability.
%
% *What is the frequency of the real pole?* 30 rad/s
% *What effect does the pole have on system stability?*
% The pole may cause instability if it is placed at a low enough frequency.
% If the pole is too slow there may not exist a gain such that a 65 deg
% phase margin can be implemented.
%
% *Does the pole have any other effect on the system?*
% Yes, the reason for the pole is that the controller can be used without
% infinite power, because the pole rolls off the magnitude response for
% higher frequencies. Any practical controller would otherwise be
% impossible because of high frequency noise that will enter any system.


%% Open Outer Loop Bode Plot
% The Open Loop Bode Plot for the Outer loop shows both the gain margin and
% the phase margin. This is useful for determining the stability of the
% system. You can see how the phase bump between the zero and the pole
% creates a region of stable phase margin from which to choose the gain.
%
% *What is the phase of your controller at low frequency?* -180 degrees
%
% *What happens to the phase when the zero is added?* At the zero frequency
% the phase is increased by 45 degrees and it adds a total of 90 phase to
% the system, most of which is added within the next decade.
%
% *What happens to the phase when the pole is added?* The pole causes a
% decrease in phase just as the zero causes an increase in phase.
figure
bode(OLTF, {0.001, 1000});
title('Open Outer Loop Bode Plot');


%% Closed Outer Loop Bode Plot
% As seen from the closed loop bode plot the system acts as a low pass
% filter. This is the ultimate tool if you need to know, given a sinusoidal
% input exactly what the output would be. The magnitude plot tells you what
% amplitude to expect relative to the input and the phase plot tells you
% how much the output will lag behind the input.
COLTF = feedback(OLTF,1);
figure
bode(COLTF, {0.01, 1000});
title('Closed Outer Loop Bode Plot');


%% Relative Stability Margins
% The chosen phase margin is very close to the actual phase margin which
% will give a good settling time. The gain margin is much higher than the
% requirement of 6dB, so this should be a relatively stable system.

% The phase margin and gain margin of the outer loop
[GMO, PMO] = margin(OLTF);
display(mag2db(GMO), 'Gain Margin 1 (dB)');
display(PMO, 'Phase Margin 1 (degrees)');


%% Step Response Performance Characteristics
% The rise time of the system is 72ms and settling time is 126ms which
% is good considering the stability and also accounting for disturbance
% rejection that has also constrained the system. The overshoot is 1.66%
% which is noticable, but tolerable considering the priority is to get the
% best settling time.

% Outer loop step response performance characteristics
COLTF = feedback(OLTF, 1);
stepinfoO = stepinfo(COLTF);
display(stepinfoO);


%% Derivations
% The derivation of the steady state error of the system to a ramp input is
% included elsewhere in this report.
%
% The derivation of the transfer function between a disturbance torque
% input and the angle output is also included elsewhere in this report.


%% Disturbance Transfer Function Bode Plot
% The bode plots of the disturbance transfer functions show how the system
% would respond to sinusoidal disturbances. These plots show that they
% behave as band-pass filters, rejecting the low and high frequencies. Note
% that the separate axes have different magnitudes, but that the phase is
% the same. This is because of the different moments of inertia for each
% axis.
Go = 1/s;
DTF1 = G1*Go/(1 + C1*G1 + C1*Co*G1*Go);
DTF2 = G2*Go/(1 + C2*G2 + C2*Co*G2*Go);
DTF3 = G3*Go/(1 + C3*G3 + C3*Co*G3*Go);
figure
bode(DTF1, DTF2, DTF3, {.0001, 10000});
title('Disturbance Transfer Functions Bode Plot');


%% Step Response of the Disturbance Transfer Function
% As discussed above on the question of where to place the zero in the
% outer loop control, there is great advantage to having a slow zero, the
% drawback is the time needed for the controller to reject constant
% disturbance. This step response of the disturbance transfer function
% shows that ~12min is needed. After that time there is no perceivable
% effect of a constant disturbance. Also note that because each axis has a
% different transfer function for the disturbance, the step response plots
% are different in magnitude.
figure
step(DTF1, DTF2, DTF3, 700);
title('Step Response of the Disturbance Transfer Function');


%% Simulink Simulation
sim('AngleControl', t_sim)
