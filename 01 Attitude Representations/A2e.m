function [ e, theta ] = A2e( A )
%A2E Convert a DCM to Eigen Axis/Angle Representation
%   Hint: Use eq 2.113, 2.114, and 2.115.  Handle theta = 0 and theta = pi.
% When theta = pi normalize the column with the largest magnitude
% When theta = 0 choose any unit vector for e (e is undefined)

% Eq 2.113
theta = acos((A(1,1)+A(2,2)+A(3,3)-1)/2);

if theta == 0
    
    e = [1; 0; 0];
    
elseif theta == pi
        
    % Eq 2.115
    t = (A + eye(3))/2;
    col = largest_col_mag(t);
    norm_factor = sqrt(t(col,col));
    e = t(:,col) / norm_factor;
        
else
    
    % Eq 2.114
    e = 1/(2*sin(theta))*[A(2,3)-A(3,2); A(3,1)-A(1,3); A(1,2)-A(2,1)];
        
end

end

function col = largest_col_mag( M )
    col = 1;
    if sum(M(:,col).^2) < sum(M(:,2).^2)
        col = 2;
    end
    if sum(M(:,col).^2) < sum(M(:,3).^2)
        col = 3;
    end
end
