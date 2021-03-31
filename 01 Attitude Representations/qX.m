function [ qout ] = qX( q, p )
%QX Calculate the product of two quaternions. (Cross Notation)
% Using equation 2.82a

qout.s = q.s*p.s - q.v'*p.v;
qout.v = q.s*p.v + p.s*q.v - X(q.v)*p.v;

end