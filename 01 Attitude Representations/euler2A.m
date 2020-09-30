function [ A ] = euler2A( e1, e2, e3, phi, theta, psi )
%EULER2A Convert a User Specified Euler Angle Sequence to a DCM
%   Hint: e1, e2, e3 specify the Euler Angle Sequence (3,2,1 is ZYX)
% phi is the angle associated with e1
% theta is the angle associated with e2
% psi is the angle associated with e3
% Create three direction cosine matrices: A1(e1,phi), A2(e2,theta), A3(e3,psi)
% A = A3*A2*A1

    A1 = xyzDCM(e1, phi);

    A2 = xyzDCM(e2, theta);

    A3 = xyzDCM(e3, psi);

    A = A3*A2*A1;

end

function [ A ] = xyzDCM(e, ang)
%XYZDCMS generate a DCM for a rotation of ang about the primary axis e
% e = 1 for the x axis, e = 2 for the y axis, e = 3 for the z axis
    if(e == 1)
        A = [1, 0, 0; 0, cos(ang), sin(ang); 0, -sin(ang), cos(ang)];
    elseif(e == 2)
        A = [cos(ang), 0, -sin(ang); 0, 1, 0; sin(ang), 0, cos(ang)];
    else % e = 3
        A = [cos(ang), sin(ang), 0; -sin(ang), cos(ang), 0; 0, 0, 1];
    end
end