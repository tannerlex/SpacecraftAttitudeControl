%% Calculate the principal open loop plant models
G1 = 1/(J_C_P(1,1)*s);
display(G1);
G2 = 1/(J_C_P(2,2)*s);
display(G2);
G3 = 1/(J_C_P(3,3)*s);
display(G3);

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
display(Kd, 'Proportional Inner loop Gains');

%% Open Inner Loop Bode Plot of each axis with the controller in the loop
% The open inner loop bode plot is exactly the same for each axis.

% Define each controller
C1 = tf(Kd1*C_pade8);
display(tf(Kd1,'OutputDelay', dt_delay));
C2 = tf(Kd2*C_pade8);
display(tf(Kd2,'OutputDelay', dt_delay));
C3 = tf(Kd3*C_pade8);
display(tf(Kd3,'OutputDelay', dt_delay));
[GM1, PM1] = margin(C1*G1*Gw);
display(mag2db(GM1), 'Gain Margin 1 (dB)');
display(PM1, 'Phase Margin 1 (degrees)');

figure;bode(C1*G1*Gw, C2*G2*Gw, C3*G3*Gw, {1,1000}, '--');grid on;
title(...
    sprintf('Open Inner Loop Bode Plot\nPhase Margin = %.2f deg, Gain Margin = %.2f dB',...
    PM1, 20*log10(GM1)));
legend('X', 'Y', 'Z');

%% Closed Inner Loop Bode Plot for each of the three axes
% As expected the inner loop, once closed with feedback, looks like a low
% pass filter.
Inner_CLTF1 = feedback(C1*G1*Gw, 1);
Inner_CLTF2 = feedback(C2*G2*Gw, 1);
Inner_CLTF3 = feedback(C3*G3*Gw, 1);
% Bandwidth is exactly the same for each axis.
Inner_BW = bandwidth(Inner_CLTF1)/(2*pi);
display(Inner_BW, 'Inner Loop bandwidth 1 (Hz)');
figure
bode(Inner_CLTF1, Inner_CLTF2, Inner_CLTF3, {1,1000}, '--'); grid on;
legend('X', 'Y', 'Z');
title(sprintf('Closed Inner Loop Bode Plots\nBandwidth = %.2f Hz',...
    Inner_BW));



%% Inner Loop Step Response Information
% The step responses are all exactly the same across the three axes. The
% settling time is a nice 0.2 seconds. Having reaction wheels in the model
% have slowed this down significantly, but it is much closer to reality.
stepinfo1 = stepinfo(Inner_CLTF1);
display(stepinfo1);
figure; step(Inner_CLTF1, Inner_CLTF2, Inner_CLTF3);
title(sprintf('Inner Loop Step Response\nPercent Overshoot = %.2f %% Rise Time = %.2f s, Settling Time = %.2f s',...
    stepinfo1.Overshoot, stepinfo1.RiseTime, stepinfo1.SettlingTime));


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

figure;bode(Outer_OLTF1, Outer_OLTF2, Outer_OLTF3, {1,1000}, '--');grid on;
title(...
    sprintf('Open Outer Loop Bode Plot\nPhase Margin = %.2f deg, Gain Margin = %.2f dB',...
    PMO1, 20*log10(GMO1)));
legend('X', 'Y', 'Z');

%% Bandwidth and Step response
Outer_CLTF1 = feedback(Outer_OLTF1,1);
Outer_CLTF2 = feedback(Outer_OLTF2,1);
Outer_CLTF3 = feedback(Outer_OLTF3,1);

Outer_BW = bandwidth(Outer_CLTF1)/(2*pi);
display(Inner_BW, 'Outer Loop bandwidth (Hz)');
figure
bode(Outer_CLTF1, Outer_CLTF2, Outer_CLTF3, {1,1000}, '--'); grid on;
legend('X', 'Y', 'Z');
title(sprintf('Closed Outer Loop Bode Plots\nBandwidth = %.2f Hz',...
    Outer_BW));

stepinfoO = stepinfo(Outer_CLTF1);
display(stepinfoO);
figure; step(Outer_CLTF1, Outer_CLTF2, Outer_CLTF3);
title(sprintf('Outer Loop Step Response\nPercent Overshoot = %.2f %% Rise Time = %.2f s, Settling Time = %.2f s',...
    stepinfoO.Overshoot, stepinfoO.RiseTime, stepinfoO.SettlingTime));
