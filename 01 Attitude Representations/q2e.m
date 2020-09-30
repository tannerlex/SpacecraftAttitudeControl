function [ e, theta ] = q2e( q )
%Q2E Convert a Quaternion to a Eigen Axis/Angle Representation
%   Hint: eq 2.124
% Calculate theta using acos
% If theta == 0 choose any unit vector
% Solve for e using q.v and sin(theta/2)
% If theta > pi, theta = 2*pi - theta, e = -e;

theta = 2 * acos(q.s);

if theta == 0

  e = [0; 0; 0];

else

  e = q.v / sin(theta / 2);

  if theta > pi

    theta = 2 * pi() - theta;
    e = -1 * e;
    
  end

end

end
