%% Linear Kalman Filter - Assignment 9
% Name: Tanner Lex Jones

clear
close all
clc

%% Set up simulation parameters
J = 0.7; % MOI
x0 = [10*pi/180; 0]; % initial state theta = 10deg, omega = 0 deg/sec

sigmaw = 0.001; % process noise
dt_sim = 0.001; % seconds between simulation times
dt_sample = 0.01; % seconds between sample times (propagate filter)

x0_hat = [0;0]; % [theta; omega]

sigmav = 1*pi/180; % measurement noise
seedv = randi(2^32);
seedw = randi(2^32);

%% Simulation to show correct propagation of mean and covariance
% These plots show the correct mean of zero and with 200 plots we may
% expect an outlier to go beyond the 3 sigma bounds, but the plots appear
% to show a very Gaussian distribution in the 3 sigma bounds.
P0 = diag([0;0]);
dt_sensor = 20; % seconds between sensor updates
tsim = 10; % simulation time in seconds

% omega plot
figure
hold on
for i = 1:200
    results = sim('kalman',tsim);
    plot(results.omega_actual)
    seedv = randi(2^32);
    seedw = randi(2^32);
end
plot(results.p3sig_o, '--m','LineWidth',2)
plot(results.m3sig_o, '--m','LineWidth',2)
title('Omega')

% theta plot
figure
hold on
for i = 1:200
    results = sim('kalman',tsim);
    plot(results.theta_actual)
    seedv = randi(2^32);
    seedw = randi(2^32);
end
plot(results.p3sig_th, '--m','LineWidth',2)
plot(results.m3sig_th, '--m','LineWidth',2)
title('Theta')


%% Simulate state estimator with updates
P0 = diag([(10*pi/180)^2;(1*pi/180)^2]);
dt_sensor = 5; % seconds between sensor updates
tsim = 60; % seconds
results = sim('kalman',tsim);

%% Show results
% The Kalman filter tracks the values of theta and omega well. It is 
% possible for the true value to be outside of the 3 sigma bounds, but this
% is unlikely as the plots show.
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

