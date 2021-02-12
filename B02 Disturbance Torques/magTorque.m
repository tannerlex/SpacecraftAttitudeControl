function Tm_B = magTorque(M0_B, B_I, q_BI)

A_BI = q2A(q_BI); % calculate DCM to convert from Inertial to Body Frames
B_B = A_BI * B_I; % local magnetic field in the Body Frame
Tm_B = cross(M0_B, B_B); % equation 3.159
end
