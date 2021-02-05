%% Orbital Environment - Assignment 1
% Name: Tanner Lex Jones

%% Preliminaries

% This cleans all variables and sets the format to display more digits.
clearvars
close all
clc
format long

%% Constants
uEarth = 3.986004418e14; % Earth Gravitational Parameter (m^3/s^2) C.104a
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

%% The position and velocity vectors projected to the inertial frame are
%% shown below.
r0_I = A_FI*rF;
v0_I = A_FI*vF;
disp(r0_I);
disp(v0_I);

%% Simulate
t_sim = 3600; % simulation time in seconds
sim('orbit', t_sim);

%% save the simulation output
orbit = ans;

%% plot of the position vector over the simulation period
figure
plot(orbit.r_I)
title('r_I: Position Vector')
figure
plot(orbit.v_I)
title('v_I: Velocity Vector')

%% plot of the magnetic field vector
figure
plot(orbit.B_I)
title('B_I: Magnetic Field Vector')

%% plot of the sun vector
figure
plot(orbit.s_I)
title('s_I: Sun Vector')

%% plot of the eclipse state
figure
plot(orbit.sun)
title('sun: Sun State (1 = not in eclipse)')

%% plot of the atmospheric density
figure
plot(orbit.rho)
title('rho: Atmospheric Density')
