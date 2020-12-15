function [th, w, wdot] = ...
    profiled_scalar(th_0, th_f, t, wdotmax, wmax)
% profiled_scalar calculates angle, velocity, and acceleration of a smooth
% slew between any two angles based on the given limitations.
% 
% inputs:
% the maximum angular velocity and acceleration.
% th_0 is the initial angle
% th_f is the final angle
% t is time
% wdotmax is the maximum angular acceleration
% wmax is the maximum angular velocity
%
% outputs:
% th is the angle at time t
% w is the angular velocity at time t
% wdot is the angular acceleration at time t

% determine the direction of travel
dir = 1;
if (th_f < th_0)
    dir = -1;
end

% calculate the highest acceptable angular velocity and acceleration
% wmax = dir*safety*hwmax/J;
% wdotmax = dir*safety*hwdotmax/J;
wmax = dir*wmax;
wdotmax = dir*wdotmax;

% determine the time it takes to get to the fastest angular velocity
t1 = wmax/wdotmax;

% calculate the amount of angular travel during acceleration periods
th_a = wdotmax*t1^2;

% determine if this is a trapezoidal or triangular profile
if (abs(th_a) >= abs(th_f))
    % triangular profile
    th_a = th_f - th_0; % all of the travel will happen during acceleration
    t1 = sqrt(th_f/wdotmax);
    
    % eliminate coasting period
    t2 = t1;
    th_c = 0;
    wmax = wdotmax*t1;
else
    % trapezoidal profile
    th_c = th_f - th_0 - th_a; % calculate angular travel during coast
    t2 = th_c/wmax + t1;
end
t3 = t2 + t1;

% determine time period
if (t < t1)
    % 1st acceleration period
    wdot = wdotmax;
    w = wdot*t;
    th = th_0 + wdot*t^2/2;
elseif (t < t2)
    % coasting time period
    wdot = 0;
    w = wmax;
    th = th_0 + th_a/2 + wmax*(t - t1);
elseif (t < t3)
    % 2nd acceleration (deceleration) period
    wdot = -wdotmax;
    w = wmax + wdot*(t - t2);
    th = th_0 + th_a/2 + th_c + (w + (wmax - w)/2)*(t - t2);
else % (t >= t3)
    % finished moving
    wdot = 0;
    w = 0;
    th = th_f; % steady as she goes
end
