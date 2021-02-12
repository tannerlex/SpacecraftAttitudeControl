function Ta_B = aeroTorque(A, Rcp, rcb_B, Un, Cd, rho, q_BI, v_I)

A_BI = q2A(q_BI); % calculate DCM to convert from/to Inertial to/from Body

Ta_B = [0; 0; 0]; % initialize torque to zero

v_B = A_BI*v_I; % velocity vector in Body frame
v_mag = norm(v_B);
uv_B = v_B/v_mag;

for i = 1 : length(A) % loop over every surface of spacecraft
    F = [0; 0; 0];
    cos_air = dot(uv_B, Un(:,i)); % Cos of Angle of Incidence of the air
    if cos_air > 0
        F = -0.5*rho*Cd(i)*A(i)*v_mag^2*cos_air*Un(:,i);
    end
    T = cross(Rcp(:,i)-rcb_B, F);
    Ta_B = Ta_B + T;
end
