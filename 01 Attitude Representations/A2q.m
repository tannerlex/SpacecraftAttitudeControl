function [q] = A2q( A )
%  A2Q Convert a Direction Cosine Matrix to Quaternion.
%   Hint: Use A2e to calculate [e, theta] then use e2q

[e, theta] = A2e(A);
q = e2q(e, theta);

end
