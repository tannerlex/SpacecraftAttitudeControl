%% Get access to slew profiler
addpath('../A09 Reaction Wheel Control')

%% Load orbit sim data
load '../B01 Orbital Environment/orbit'
r_I = orbit.r_I;
v_I = orbit.v_I;
s_I = orbit.s_I;
sun = orbit.sun;
rho = orbit.rho;
B_I = orbit.B_I;
A_IE = orbit.A_IE; % DCM from Earth Fixed to Inertial

%% Run Mission Planning Simulation
% Find the quaternions: pointing at the sun & pointing at the ground track
plan = sim('missionplanning', t_sim);
angle_zenith = plan.angle_zenith;
t = angle_zenith.time; % seconds
qs_BI = plan.qs_BI;
ql_BI = plan.ql_BI;

% plot the zenith angle
figure
plot(angle_zenith)
title('Zenith Angle')
ylabel('$\theta_{zenith}(t)$', 'fontsize', 14, 'Interpreter','latex');


%% Find the angle command switch times

% angle above horizon at which sat must be pointing at ground target
abvHorizon = 10; % degrees
maxSlewTime = 21; % seconds
upper_zenith = 90 - abvHorizon;
startGndTrk = 0;
startGndTrkIdx = 0;
stopGndTrk = 0;
stopGndTrkIdx = 0;
for i = 1:(size(angle_zenith.data,1) - 1) 
    if ((angle_zenith.data(i) > upper_zenith) && ...
        (angle_zenith.data(i+1) <= upper_zenith))
        startGndTrk = t(i);
        startGndTrkIdx = i;
    elseif ((angle_zenith.data(i) <= upper_zenith) && ...
        (angle_zenith.data(i+1) > upper_zenith))
        stopGndTrk = t(i);
        stopGndTrkIdx = i;
    end
end

% time at which to start slew between sun pointing and ground tracking
switch2gndtrk = startGndTrk - maxSlewTime;
switch2gndtrkIdx = 0;
for i = 1:(startGndTrkIdx - 1)
    if ((t(i) <= switch2gndtrk) && ...
        (t(i+1) > switch2gndtrk))
        switch2gndtrkIdx = i;
    end
end

% time at which to stop slew between ground track and sun pointing
resSunPoint = stopGndTrk + maxSlewTime;
resSunPointIdx = 0;
i = stopGndTrkIdx;
while 0 == resSunPointIdx
    i = i + 1;
    if (t(i) > resSunPoint)
        resSunPointIdx = i;
    end
end

slewtimes = [switch2gndtrk, startGndTrk, stopGndTrk, resSunPoint];
