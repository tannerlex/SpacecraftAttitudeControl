clearvars
close all
clc
format long

%% Constants
uEarth = 3.986004418e14; % Earth Gravitational Parameter (m^3/s^2) eq C.104
rEarth = 6.378137e6; % Earth Mean Equatorial Radius (m) eq C.104b

%% Orbital Parameters
a = rEarth + 500e3; % 500 km orbit (m)
e = 0.01; % Circular Orbit
theta = -30*pi/180; % True Anomaly (rad)
Omega = -10*pi/180; % Right Ascension of Ascending Node (rad)
I = 45*pi/180; % Inclination (rad)
w = 20*pi/180; % Argument of Perigee (rad)
% r_p = a*(1-e); % distance at perigee (eq C.14a)
% r_a = a*(1+e); % distance at apogee (eq C.14b)
p = a*(1-e^2); % perifocal position (eq C.15)
r = p/(1+e*cos(theta)); % Distance (eq C.21)

%% Initial Satellite Position and Velocity
cO=cos(Omega);
cw=cos(w);
cI=cos(I);
cth=cos(theta);
sO=sin(Omega);
sw=sin(w);
sI=sin(I);
sth=sin(theta);
rF = [r*cth; r*sth; 0]; % (eq C.49)
vF = [-sqrt(uEarth/p)*sth; sqrt(uEarth/p)*(e+cth); 0]; % (eq C.76)
A_IF = [cO*cw-cI*sO*sw, cw*sO+cO*cI*sw, sI*sw; ...
       -cO*sw-cI*cw*sO, cO*cI*cw-sO*sw, cw*sI; ...
                 sO*sI,         -cO*sI,    cI];
A_FI = A_IF';
r0_I = A_FI*rF;
v0_I = A_FI*vF;
