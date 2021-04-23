function [qp_BI, Pp, Bwp] = update(qm_BI, Pm, Bwm, v_B, v_I, sigmav)
%UPDATE updates the Kalman filter
%   q-_BI is the previous attitude quaternion
%   P- is the previous covariance
%   Bw- is the previous gyro bias
%   v_B is the vector expressed in the body frame
%   v_I is the vector expressed in the inertial frame
%   sigmav is the STDev of the measurement noise

Ahat_BI = q2A(qm_BI); % calculate DCM

vhat_B = Ahat_BI*v_I; % project inertial vector to the body frame

H = [X(vhat_B), zeros(3, 3)]; % Equation 6.53

R = sigmav^2*eye(3); % Equation 6.48b

K = Pm*H'/(H*Pm*H' + R); % calculate the Kalman gain - Equation E.126

dx = K*(v_B - vhat_B); % gain multiplied by residual

dth = dx(1:3); % extract error rotation vector
dBw = dx(4:6); % extract change in bias

% calculate new attitude using Equation 6.27
qp_BI.s = qm_BI.s - 1/2*(qm_BI.v')*dth;
qp_BI.v = qm_BI.v + 1/2*(qm_BI.s*eye(3) + X(qm_BI.v))*dth;
qp_BI = qUnit(qp_BI); % normalize (Eq 6.28)

Pp = (eye(6) - K*H)*Pm; % update covariance matrix
Pp = (Pp + Pp')/2; % make sure covariance matrix is symmetrical

Bwp = Bwm + dBw; % updated bias (from Table 6.3)

end
