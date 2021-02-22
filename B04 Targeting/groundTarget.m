function [ql_BI, angle_zenith] = groundTarget(u_I, rl_I, r_I, v_I)

% Vector from Logan to Satellite
rsl_I = r_I - rl_I;

% Angle between Logan "Up" vector and Satellite
angle_zenith = 180/pi*acos(rsl_I'/norm(rsl_I)*u_I);

% Orbital Angular Momentum Vector
h_I = cross(r_I, v_I);
h_I = h_I/norm(h_I);

% Point Z Body Axis at Logan
z_I = -rsl_I/norm(rsl_I);

% X Body Axis
x_I = cross(h_I, z_I);
x_I = x_I/norm(x_I);

% Y Body Axis completes triad (close to h_I)
y_I = cross(z_I, x_I);

% Direction Cosine Matrix (DCM) from Basis Vectors
A_BI = [x_I'; y_I'; z_I'];

% Quaternion from DCM
ql_BI = A2q(A_BI);
