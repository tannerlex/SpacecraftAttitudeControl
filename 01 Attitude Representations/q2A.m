function [A] = q2A( q )
%  Q2A Convert a Quaternion to Direction Cosine Matrix.
%   Hint: Compare eq 2.125 and your X function

A = (q.s^2 - sum(q.v.^2)) * eye(3) - 2*q.s*X(q.v) + 2*q.v*q.v';

end