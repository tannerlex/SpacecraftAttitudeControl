clear
close all
clc

J = 0.7; % MOI

x0 = [10*pi/180; 0]; % initial state theta = 10deg, omega = 0 deg/sec


sigmaw = 0.001; % process noise
dt_sim = 0.001;

dt_sample = 0.01;
dt_sensor = 5;

x0_hat = [0;0]; % [theta; omega]
P0 = diag([(10*pi/180)^2;(1*pi/180)^2]);

sigmav = 1*pi/180; % measurement noise
seedv = randi(2^32);
seedw = randi(2^32);

%% Simulate
tsim = 60; % seconds
results = sim('kalman',tsim);

%% Show results
figure
plot(results.theta_actual)
hold on
plot(results.theta_est)
plot(results.p3sig_th)
plot(results.m3sig_th)
legend('Theta','Theta Estimation','+3 sigma','-3 sigma')

figure
plot(results.omega_actual)
hold on
plot(results.omega_est)
plot(results.p3sig_o)
plot(results.m3sig_o)
legend('Omega','Omega Estimated','+3 sigma','-3 sigma')

