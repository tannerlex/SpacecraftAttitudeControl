function [ phi, theta, psi ] = A2zyx( A )
%A2zyx Convert a DCM to a ZYX (phi, theta, psi) Euler Angle Sequence
%   Hint: Use Table B.2 in appendix B, or derive the DCM using euler2A
% Use asin for theta, and atan2 for phi and psi
% phi is the angle associated with the z rotation
% theta is the angle associated with the y rotation
% psi is the angle associated with the x rotation

    phi = atan2(A(1,2), A(1,1));
    theta = -asin(A(1,3));
    psi = atan2(A(2,3), A(3,3));

end

