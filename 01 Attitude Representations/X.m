function [ vX ] = X( v )
% X() returns a 3x3 Cross Product Matrix
% input vector must be of length 3
  vX = zeros(3);
  vX(1,2) =-v(3);
  vX(1,3) = v(2);
  vX(2,1) = v(3);
  vX(2,3) =-v(1);
  vX(3,1) =-v(2);
  vX(3,2) = v(1);
end
