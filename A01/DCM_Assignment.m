%% 01 Cross Product Matrix and Direction Cosine Matrices
% Name: Tanner Lex Jones

%% Preliminaries
% This cleans all variables and sets the format to display more digits.
clearvars
close all
clc
format long

%% Satellite Orbit
% A satellite is in orbit with the following position and velocity vectors
% relative to an inertial frame and projected to the inertial frame:
r_I = [6878.06; -28.28; -12.28]; % km
v_I = [0.031; 5.38; 5.38]; % km/s

%% Orbital Reference Frame / Local Vertical Local Hortizonal (L Frame)
% The satellite is designed to be a "Nadir" pointing satellite and needs to
% align its axes with an orbital reference frame.  The chosen frame has its
% z axis pointed at the Earth, and its y axis in the negative direction of
% the orbital angular momentum vector.  What is the direction cosine matrix
% that desribes the rotation to this frame from the inertial frame given
% the current position and velocity vector?
yI_L = X(-1*r_I)*v_I / norm(X(r_I)*v_I);
zI_L = -1 * r_I / norm(r_I);
xI_L = X(yI_L) * zI_L;

% Calculate the Direction Cosine Matrix A_LI
A_LI = [xI_L'; yI_L'; zI_L'];
disp("A_LI:");
disp(A_LI);

% Check that this is an ortho normal matrix
A_IL = A_LI';
eps = 0.000001;
if all(((A_LI * A_IL) - eye(3)) < abs(eps * ones(3)))
  disp('A_LI is an orthonormal matrix.');
else
  disp('CALCULATION ERROR: A_LI is not orthonormal!');
end

%% Projecting a vector from Frame I to Frame L
% If a vector v1 has the following representation in Frame I:
%
% * v1_I = [1;0;0];
%
% What is its representation in Frame L (v1_L)?
v1_I = [1; 0; 0];

% Calculate v1 represented in the local level frame
v1_L = A_LI * v1_I;
disp("v1_L");
disp(v1_L);

%% Projecting a vector from Frame L to Frame I
% If a vector v2 has the following representation in Frame L:
%
% * v2_L = [1;0;0];
%
% What is its representation in Frame I (v2_I)?
v2_L = [1; 0; 0];

% Calculate v2 represented in the inertial frame
v2_I = A_IL * v2_L;
disp("v2_I:");
disp(v2_I);
