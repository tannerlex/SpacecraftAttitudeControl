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
        switch2gndtrkIdx = i;
    end
end

% time at which to stop slew between ground track and sun pointing
resSunPoint = stopGndTrk + maxSlewTime
resSunPointIdx = 0;
i = stopGndTrkIdx;
while 0 == resSunPointIdx
    i = i + 1;
    if (t(i) > resSunPoint)
        resSunPointIdx = i;
    end
end

%% Solve for the commanded quaternions throughout the mission.
% 1st period is sun tracking, use qs_PI
% 2nd period is a slew between sun tracking and ground tracking
% 3rd period is ground point tracking, use ql_PI
% 4th period is a slew from the ground track to the sun
% 5th period is sun tracking, use qs_PI

%% initialize variables used for pointing commands
% qstar_PI gets initialized to qs_PI (covers 1st & 5th periods)
% w and wdot get initialized to zero
qstar_BI = qs_BI; % initialize to the sun pointing angle
% qstar_PI = timeseries(q0_BI,t);
qs_PI = qs_BI;
for i = 1:length(t)
    %convert sun pointing from Body to Principle
    qsun_BI.s = qs_BI.s.data(i);
    qsun_BI.v = qs_BI.v.data(i,:);
    q = qUnit(A2q(A_PB*q2A(qsun_BI)));
    qs_PI.v.data(i,:) = q.v;
    qs_PI.s.data(i) = q.s;
    %convert USU pointing from Body to Principle
    qusu_BI.s = ql_BI.s.data(i);
    qusu_BI.v = ql_BI.v.data(i,:);
    q = qUnit(A2q(A_PB*q2A(qusu_BI)));
    ql_PI.v.data(i,:) = q.v;
    ql_PI.s.data(i) = q.s;
end
qstar_PI = qs_PI; % initialize to the sun pointing angle
wbidotstar_P = timeseries(zeros(length(t),1),t);
wbistar_P = timeseries(zeros(length(t),1),t);

%% 2nd period: determine slew command between sun and ground track
q0_PIslew.v = qs_PI.v.data(switch2gndtrkIdx,:)';
q0_PIslew.s = qs_PI.s.data(switch2gndtrkIdx);
qf_PIslew.v = ql_PI.v.data(startGndTrkIdx,:)';
qf_PIslew.s = ql_PI.s.data(startGndTrkIdx);
q0_IPslew = qUnit(A2q(q2A(q0_PIslew)'));
qe = qUnit(qX(qf_PIslew, q0_IPslew)); % error quaternion
[ehat, th_f] = q2e(qe);
th_0 = 0;
for i = switch2gndtrkIdx:(startGndTrkIdx-1)
    [th, w, wdot] = ...
        profiled_scalar(th_0, th_f, t(i)-switch2gndtrk, wdotmax, wmax);
    qr = qUnit(e2q(ehat, th));
    qs = qUnit(qX(qr, q0_PIslew));
    qstar_PI.s.data(i) = qs.s;
    qstar_PI.v.data(i,:) = qs.v;
    wbistar_P.data(i) = w;
    wbidotstar_P.data(i) = wdot;
end

%% 3rd period: track ground track
for i = startGndTrkIdx:stopGndTrkIdx
    qstar_PI.v.data(i,:) = ql_PI.v.data(i,:);
    qstar_PI.s.data(i) = ql_PI.s.data(i);
end

%% 4th period: determine slew command between ground track and sun
q0_PIslew.v = ql_PI.v.data(stopGndTrkIdx,:)';
q0_PIslew.s = ql_PI.s.data(stopGndTrkIdx);
qf_PIslew.v = qs_PI.v.data(resSunPointIdx,:)';
qf_PIslew.s = qs_PI.s.data(resSunPointIdx);
q0_IPslew = qUnit(A2q(q2A(q0_PIslew)'));
qe = qUnit(qX(qf_PIslew, q0_IPslew)); % error quaternion
[ehat, th_f] = q2e(qe);
th_0 = 0;
for i = stopGndTrkIdx:(resSunPointIdx-1)
    [th, w, wdot] = ...
        profiled_scalar(th_0, th_f, t(i)-stopGndTrk, wdotmax, wmax);
    qr = qUnit(e2q(ehat, th));
    qs = qUnit(qX(qr, q0_PIslew));
    qstar_PI.s.data(i) = qs.s;
    qstar_PI.v.data(i,:) = qs.v;
    wbistar_P.data(i) = w;
    wbidotstar_P.data(i) = wdot;
end


%% Convert from principle frame to body frame for animation
for i = 1:length(t)
    q_PI.s = qstar_PI.s.data(i);
    q_PI.v = qstar_PI.v.data(i,:);
    q = qUnit(A2q(A_BP*q2A(q_PI)));
    qstar_BI.v.data(i,:) = q.v;
    qstar_BI.s.data(i) = q.s;
end
plot(qstar_BI.v)
hold on
plot(qstar_BI.s)
sim('animation', t_sim)
