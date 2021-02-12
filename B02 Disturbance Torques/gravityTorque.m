function Tg_B = gravityTorque(J_C_P, A_PB, uEarth, r_I, v_I, q_BI)

% Declare the Orbital Reference frame
zhat_IR = -r_I/norm(r_I);
yhat_IR = cross(-r_I, v_I)/norm(cross(-r_I, v_I));
xhat_IR = cross(yhat_IR, zhat_IR);
A_RI = [xhat_IR'; yhat_IR'; zhat_IR'];
A_IR = A_RI';
A_BI = q2A(q_BI); % calculate DCM to convert from Inertial to Body Frames
A_PR = A_PB * A_BI * A_IR;
A_BP = A_PB';

% Calculate gravity torque vector using equation 3.157
J1 = J_C_P(1,1);
J2 = J_C_P(2,2);
J3 = J_C_P(3,3);
M = [(J3-J2) * A_PR(2,3) * A_PR(3,3);
     (J1-J3) * A_PR(1,3) * A_PR(3,3);
     (J2-J1) * A_PR(1,3) * A_PR(2,3)];
Tg_P = 3*uEarth/norm(r_I)^3 * M;
Tg_B = A_BP * Tg_P;
