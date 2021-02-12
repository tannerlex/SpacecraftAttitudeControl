% Area, Vector to center of pressure from origin, unit normal vector
% Drag Coefficient, and surface reflectivity of panel 1
A1 = 0.03275; % m^2
rcp1_B = [0.16364; -0.03142;  0.14239]; % m
un1_B = [0;1;0];
cd1 = 2.2;
cr1 = 2.0;

% Area, Vector to center of pressure from origin, unit normal vector
% Drag Coefficient, and surface reflectivity of panel 2
A2 = 0.03275; % m^2
rcp2_B = [0.16364; -0.13142;  0.14239]; % m
un2_B = [0;-1;0];
cd2 = 2.2;
cr2 = 2.0;

% Area, Vector to center of pressure from origin, unit normal vector
% Drag Coefficient, and surface reflectivity of panel 3
A3 = 0.03275; % m^2
rcp3_B = [0.21364; -0.08142;  0.14239]; % m
un3_B = [1;0;0];
cd3 = 2.2;
cr3 = 2.0;

% Area, Vector to center of pressure from origin, unit normal vector
% Drag Coefficient, and surface reflectivity of panel 4
A4 = 0.03275; % m^2
rcp4_B = [0.11364; -0.08142;  0.14239]; % m
un4_B = [-1;0;0];
cd4 = 2.2;
cr4 = 2.0;

% Area, Vector to center of pressure from origin, unit normal vector
% Drag Coefficient, and surface reflectivity of panel 5
A5 = 0.01; % m^2
rcp5_B = [0.16364; -0.08142;  0.30614]; % m
un5_B = [0;0;1];
cd5 = 2.2;
cr5 = 2.0;

% Area, Vector to center of pressure from origin, unit normal vector
% Drag Coefficient, and surface reflectivity of panel 6
A6 = 0.01; % m^2
rcp6_B = [0.16364; -0.08142; -0.02136]; % m
un6_B = [0;0;-1];
cd6 = 2.2;
cr6 = 2.0;

% Area, Vector to center of pressure from origin, unit normal vector
% Drag Coefficient, and surface reflectivity of panel 7
A7 = 0.026363; % m^2
rcp7_B = [0.16364;  0.082247415075738; -0.135027415075738]; % m
un7_B = [0; 0.707106781186548; 0.707106781186548];
cd7 = 2.2;
cr7 = 2.0;

% Area, Vector to center of pressure from origin, unit normal vector
% Drag Coefficient, and surface reflectivity of panel 8
A8 = 0.026363; % m^2
rcp8_B = [0.16364;  0.081830222074837; -0.135444608076638]; % m
un8_B = [0; -0.707106781186548; -0.707106781186548];
cd8 = 2.2;
cr8 = 2.0;

% Combine Panel Parameters into Matrices for Import into Simulink
A = [A1,A2,A3,A4,A5,A6,A7,A8];
Rcp = [rcp1_B,rcp2_B,rcp3_B,rcp4_B,rcp5_B,rcp6_B,rcp7_B,rcp8_B];
Un = [un1_B,un2_B,un3_B,un4_B,un5_B,un6_B,un7_B,un8_B];
Cd = [cd1,cd2,cd3,cd4,cd5,cd6,cd7,cd8];
Cr = [cr1,cr2,cr3,cr4,cr5,cr6,cr7,cr8];
