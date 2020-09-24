%% Attitude Representations Assignment
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
ri_I = [6878.06; -28.28; -12.28]; % km
vi_I = [0.031; 5.38; 5.38]; % km/s

%% Satellite Attitude
% The attitude of the satellite is given as the quaternion that describes
% the rotation to the body frame from the inertial frame as a structure.
q_BI.s = 0.605975076083538;
q_BI.v = [-0.687258217268287
          -0.357452241107369
          0.180826561351166];
%%
% The quaternion is normalized to ensure it is a unit quaternion
q_BI = qUnit(q_BI);

%% Orbital Reference Frame / Local Vertical Local Hortizonal (L Frame)
% The satellite is designed to be a "Nadir" pointing satellite and needs to
% align its axes with an orbital reference frame.  The chosen frame has its
% z axis pointed at the Earth, and its y axis in the negative direction of
% the orbital angular momentum vector.  What is the direction cosine matrix
% that desribes the rotation to this frame from the inertial frame given
% the current position and velocity vector?
zl_I = -1 * ri_I / norm(ri_I);
yl_I = X(-1 * ri_I) * vi_I / norm(X(ri_I) * vi_I);
xl_I = X(yl_I) * zl_I;

A_LI = [xl_I'; yl_I'; zl_I'];
disp('A_LI = ');
disp(A_LI);

%%
% What is the quaternion representation of A_LI?  Use your A2q function.
% Remember to normalize the quaternion.
q_LI = A2q(A_LI);
q_LI = qUnit(q_LI);
disp('q_LI.s = ');
disp(q_LI.s);
disp('q_LI.v = ');
disp(q_LI.v);

%% Euler Angle Sequence
% What is the ZYX Euler Angle Sequence that describes the rotation from the
% Orbital Reference Frame to the Body Frame?  First solve for q_BL using
% your qX and qT functions.  Remember to normalize the quaternion.  Then 
% convert q_BL to A_BL using your q2A function.  Finally convert A_BL to a 
% ZYX Euler Angle Sequence using your A2zyx function.
q_BL = qX(q_BI, qT(q_LI));
q_BL = qUnit(q_BL);
A_BL = q2A(q_BL);

[phi, theta, psi] = A2zyx(A_BL);
disp('phi = ');
disp(phi*180/pi);
disp('theta = ');
disp(theta*180/pi);
disp('psi = ');
disp(psi*180/pi);

%%
% There was a miscommunication between yourself and another engineer and
% they thought an XYZ Euler Angle sequence was to be used rather than a
% ZYX.  What would the resultant DCM be if phi, theta, and psi from above
% were used in an XYZ Euler Angle Sequence?  Use your euler2A function.
Awrong_BL = euler2A(1, 2, 3, phi, theta, psi);
disp('Awrong_BL = ');
disp(Awrong_BL);

%% Euler Axis/Angle Representation
% The satellite needs to perform a slew maneuver to align its axes with the
% orbital reference frame.  The shortest possible maneuver is to slew about
% the eigen axis of the DCM which describes the rotation from the body
% frame to the orbital reference frame.  Calculate this axis and the angle
% through which the satellite needs to rotate.  Perform the calculation
% using both your A2e function and your q2e function.  Be careful about
% the direction of the rotation
A_LB = A_BL';
q_LB = qT(q_BL);

[e, angle] = A2e(A_LB);
disp('angle and axis from A2e = ');
disp([angle*180/pi;e]);

[e, angle] = q2e(q_LB);
disp('angle and axis from q2e = ');
disp([angle*180/pi;e]);

%%
% What is the direction cosine matrix representation of this axis and
% angle?  Use your e2A function.
A_LB = e2A(e, angle);
disp('A_LB from e2A = ');
disp(A_LB);

%%
% What is the quaternion representation of this axis and angle?  Use your
% e2q function.
q_LB = e2q(e, angle);
disp('q_LB.s from e2q = ');
disp(q_LB.s);
disp('q_LB.v from e2q = ');
disp(q_LB.v);

%% Rotation Vector
% What is the rotation vector that describes the rotation from the B frame
% to the L frame?  Use the same axis and angle from above.
theta_LB = angle * e;
disp('theta_LB = ');
disp(theta_LB);
