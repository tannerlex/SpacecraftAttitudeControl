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

tsim = 60; % seconds

sim('kalman',tsim);

