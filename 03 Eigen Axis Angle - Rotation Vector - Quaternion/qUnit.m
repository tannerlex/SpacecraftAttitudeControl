function [ qout ] = qUnit( q )
%QNORMALIZE Normalizes a quaternion
%   Hint: Include the vector and scalar portions
norm_factor = sqrt(sum(q.v .^ 2)+q.s^2);
qout.s = q.s / norm_factor;
qout.v = q.v ./ norm_factor;

end
