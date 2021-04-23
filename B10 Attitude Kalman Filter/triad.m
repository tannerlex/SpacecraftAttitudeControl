function [A_BI] = triad(v1_B, v1_I, v2_B, v2_I)
%TRIAD Implements the triad algorithm to calculate a direction cosine
%matrix (DCM) representing the rotation of the body frame relative to the
%inertial frame.
%   This function accepts 2 vectors both in the body and inertial frames,
%   prioritizing the first vector measurements and returns the calculated
%   DCM.
u1_B = v1_B/norm(v1_B);
u1_I = v1_I/norm(v1_I);

u2_B = cross(u1_B, v2_B);
u2_B = u2_B/norm(u2_B);
u2_I = cross(u1_I,v2_I);
u2_I = u2_I/norm(u2_I);

u3_B = cross(u1_B, u2_B);
u3_I = cross(u1_I, u2_I);

A_BI = u1_B*u1_I' + u2_B*u2_I' + u3_B*u3_I';
end
