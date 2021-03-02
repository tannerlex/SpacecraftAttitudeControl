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

q0_BI.s = 1;
q0_BI.v = [0;0;0];

%% Run Mission Planning Simulation
% Find the quaternions: pointing at the sun & pointing at the ground track
plan = sim('missionplanning', t_sim);
angle_zenith = plan.angle_zenith;
t = angle_zenith.time; % seconds
qs_BI = plan.qs_BI;
ql_BI = plan.ql_BI;

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
        startGndTrk = t(i)
        startGndTrkIdx = i;
    elseif ((angle_zenith.data(i) <= upper_zenith) && ...
        (angle_zenith.data(i+1) > upper_zenith))
        stopGndTrk = t(i)
        stopGndTrkIdx = i;
    end
end

% time at which to start slew between sun pointing and ground tracking
switch2gndtrk = startGndTrk - maxSlewTime
switch2gndtrkIdx = 0;
for i = 1:(startGndTrkIdx - 1)
    if ((t(i) <= switch2gndtrk) && ...
        (t(i+1) > switch2gndtrk))
        switch2gndtrkIdx = i
    end
end

% time at which to stop slew between ground track and sun pointing
resSunPoint = stopGndTrk + maxSlewTime
resSunPointIdx = 0;
i = stopGndTrkIdx;
while 0 == resSunPointIdx
    i = i + 1;
    if (t(i) > resSunPoint)
        resSunPointIdx = i
    end
end

%% Solve for the commanded quaternion throughout the mission.

% initialize variables used for pointing commands
qstar_BI = qs_BI; % initialize to the sun pointing angle
wbidotstar_P = timeseries(zeros(length(t),1),t);
wbistar_P = timeseries(zeros(length(t),1),t);

% determine slew command between sun and ground track
q0_BIslew.v = qs_BI.v.data(switch2gndtrkIdx,:);
q0_BIslew.s = qs_BI.s.data(switch2gndtrkIdx);
qf_BIslew.v = ql_BI.v.data(startGndTrkIdx,:);
qf_BIslew.s = ql_BI.s.data(startGndTrkIdx);
[~, th_0] = q2e(q0_BIslew);
[e, th_f] = q2e(qf_BIslew);
for i = switch2gndtrkIdx:(startGndTrkIdx-1)
    [th, w, wdot] = ...
        profiled_scalar(th_0, th_f, t(i), wdotmax, wmax);
    q = e2q(e, th);
    qstar_BI.s.data(i) = q.s;
    qstar_BI.v.data(i,:) = q.v;
    wbistar_P.data(i) = w;
    wbidotstar_P.data(i) = wdot;
end

% track ground track
for i = startGndTrkIdx:stopGndTrkIdx
    qstar_BI.v.data(i,:) = ql_BI.v.data(i,:);
    qstar_BI.s.data(i) = ql_BI.s.data(i);
end

% determine slew command between ground track and sun
q0_BIslew.v = ql_BI.v.data(stopGndTrkIdx,:);
q0_BIslew.s = ql_BI.s.data(stopGndTrkIdx);
qf_BIslew.v = ql_BI.v.data(resSunPointIdx,:);
qf_BIslew.s = ql_BI.s.data(resSunPointIdx);
[~, th_0] = q2e(q0_BIslew);
[e, th_f] = q2e(qf_BIslew);
for i = stopGndTrkIdx:(resSunPointIdx-1)
    [th, w, wdot] = ...
        profiled_scalar(th_0, th_f, t(i), wdotmax, wmax);
    qstar_BI.s.data(i) = q.s;
    qstar_BI.v.data(i,:) = q.v;
    wbistar_P.data(i) = w;
    wbidotstar_P.data(i) = wdot;
end

% sim('animation', t_sim)
