function eclipse = get_eclipse(r_I, rEarth, s_I)

e_es = s_I/norm(s_I); % calculate the sun unit vector
eclipse = dot(r_I, e_es) < -sqrt(r_I'*r_I-rEarth^2); % equation D.57

end
