function s_I = get_sunvector(JD)

% start with (eq D.49)
T_UT1 = (JD-2451545)/36525; % 

phi_s = 280.460 + 36000.771 * T_UT1; % mean longitude of the sun (D.48a)
M_s = 357.5277233 + 35999.05034 * T_UT1; % mean anomaly of the sun (D.48b)

% longitude of the ecliptic (eq D.50)
phi_ecl = phi_s + 1.914666471 * sind(M_s) + 0.019994643 * sind(2*M_s);
% obliquity of the ecliptic (eq D.51)
eps_ecl = 23.439291 - 0.0130042 * T_UT1;

% unit vector from Earth to Sun (eq D.52)
e_es = [              cosd(phi_ecl); ...
        cosd(eps_ecl)*sind(phi_ecl); ...
        sind(eps_ecl)*sind(phi_ecl)];

% distance in AU between the Earth and Sun (eq D.53)
r_e2s = 1.000140612 - 0.016708617*cosd(M_s) - 0.000139589*cosd(2*M_s);

% position vector in meters from Earth to Sun
s_I_AU = r_e2s * e_es;

s_I = s_I_AU * 149597870700; % sun vector in meters

end
