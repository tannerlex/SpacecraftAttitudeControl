function Fsr = solarRadiationForce(Psr, C_R, A, s_B, n_B, sun)
Fsr = [0; 0; 0];
sdotn = dot(s_B, n_B);
if sdotn > 0 && sun
    Fsr = -Psr * C_R * A * sdotn * n_B;
end
