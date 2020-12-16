close all
clc
hwdotmax = 0.004;
hwmax = 0.015;
safety = 0.5; % reaction wheel safety factor
J = 0.039943262702511;
wmax = safety*hwmax/J;
wdotmax = safety*hwdotmax/J;
t = 0:.01:30;
th0 = 90*pi/180;
thf = -90*pi/180;
theta = zeros(1,length(t));
w = zeros(1,length(t));
wdot = zeros(1,length(t));
for i = 1:length(t)
    [theta(i), w(i), wdot(i)] = ...
        profiled_scalar(th0, thf, t(i), wdotmax, wmax);
end

figure
plot(t,theta)
figure
plot(t,w)
figure
plot(t,wdot)
