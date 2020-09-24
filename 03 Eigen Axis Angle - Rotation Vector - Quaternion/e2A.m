function [ A ] = e2A( e, theta )
%E2A Convert a Eigen Axis/Angle Representation to a DCM
%   Hint: Use eq 2.109 and your cross product matrix function (X)

A = eye(3) - sin(theta) * X(e) + (1 - cos(theta)) * X(e)^2;

end
