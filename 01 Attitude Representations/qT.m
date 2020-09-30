function [qout] = qT( q ) 
%  QT Calculate the conjugate of a quaternion.
%   Hint: Eq 2.91.  Store quaternion as structure.

qout.s = q.s;
qout.v = -1 * q.v;

end
