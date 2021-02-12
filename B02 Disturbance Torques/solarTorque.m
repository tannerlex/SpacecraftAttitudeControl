function Ts_B= solarTorque(A, Rcp, rcb_B, Psr, Cr, Un, q_BI, sun, s_I, r_I)

A_BI = q2A(q_BI); % calculate DCM to convert from/to Inertial to/from Body

Ts_B = [0; 0; 0]; % initialize torque to zero


s_B = A_BI * (s_I - r_I); % sun vector from satellite in Body frame

for i = 1 : length(A) % loop over every surface of spacecraft
    F = solarRadiationForce(Psr, Cr(i), A(i), s_B, Un(:,i), sun);
    T = cross(Rcp(:,i) - rcb_B, F);
    Ts_B = Ts_B + T;
end
