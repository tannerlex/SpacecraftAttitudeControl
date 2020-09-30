function [ q ] = e2q( e, theta )
%E2Q Convert an Euler Axis/Angle Representation to a Quaternion
%   Hint: Use eq 2.124
% Store as a structure q.s = scalar, q.v = vector

q.s = cos(theta / 2);
q.v = sin(theta / 2) * e;

end
