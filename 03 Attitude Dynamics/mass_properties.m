%% Satellite Geometry
% The position vector from the origin of the B frame to the link between
% the structure and the deployable.
rlb_B = [0.16364; -0.03142; -0.02136]; % m

%% Satellite Mass Properties
% The mass properties of the satellite will be extracted from the Solid
% Works Assembly.  The mass properties of the structure and deployable will
% be extracted separately and combined in this script.

%% Mass properties of Satellite Structure

%%
% Mass of satellite structure
ms = 3.21000120; % kg

%%
% Position vector to center of mass of satellite structure relative to
% origin of the B frame projected to the basis vectors of the B frame.
rsb_B = [0.16439646; -0.08187950; 0.09592783]; % m

%%
% Moment of inertia tensor of satellite structure about the origin of the B
% frame projected to the basis vectors of the B frame.
Jxx = 0.08636806;
Jxy = -0.04320451;
Jxz = 0.05087768;
Jyy = 0.15161865;
Jyz = -0.02530725;
Jzz = 0.11387280;

Js_B_B = [ Jxx -Jxy -Jxz
          -Jxy  Jyy -Jyz
          -Jxz -Jyz  Jzz]; % kg*m^2

%%
% Moment of inertia tensor of the satellite structure about the center of
% mass of the structure projected to the basis vectors of the B frame.
Js_S_B = Js_B_B - ms*(rsb_B'*rsb_B*eye(3)-rsb_B*rsb_B'); % kg*m^2

%%
% Extract MOI and POI into vectors for SimScape.
Ms_S_B = diag(Js_S_B); % Moments of inertia of the structure
Ps_S_B = [Js_S_B(2,3);
          Js_S_B(1,3);
          Js_S_B(1,2)]; % Products of inertia of the structure
       
%% Mass properties of Deployable Panel

%%
% Mass of deployable panel
md = 0.05290847; % kg

%%
% Position vector to center of mass of deployable panel relative to
% origin of the B frame projected to the basis vectors of the B frame.
rdb_B = [0.16364135; 0.08256652; -0.13481916]; % m

%%
% Moment of inertia tensor of deployable panel about the origin of the B
% frame projected to the basis vectors of the B frame.
Jxx = 0.00177584;
Jxy = 0.00071486;
Jxz = -0.00116727;
Jyy = 0.00263292;
Jyz = -0.00081569;
Jzz = 0.00203193;

Jd_B_B = [ Jxx -Jxy -Jxz
          -Jxy  Jyy -Jyz
          -Jxz -Jyz  Jzz]; % kg*m^2

%%
% Moment of inertia tensor of the deployable panel about the center of
% mass of the deployable projected to the basis vectors of the B frame.
Jd_D_B = Jd_B_B - md*(rdb_B'*rdb_B*eye(3)-rdb_B*rdb_B'); % kg*m^2 

%%
% Extract MOI and POI into vectors for SimScape.
Md_D_B = diag(Jd_D_B); % Moments of inertia of the deployable
Pd_D_B = [Jd_D_B(2,3);
          Jd_D_B(1,3);
          Jd_D_B(1,2)]; % Products of inertia of the deployable  

%% Combined Mass Properties

%%
% Position vector to combined center of mass of the satellite structure
% and deployable panel relative to the origin of the B frame projected to
% the basis vectors of the B frame.  CALCULATE IT
rcb_B = (ms*rsb_B + md*rdb_B) / (ms+md); % m

%%
% Moment of inertia tensor of the satellite structure about combined center
% of mass projected to the basis vectors of the B frame.
rcs_B = rcb_B - rsb_B;
Js_C_B = Js_S_B + ms*(rcs_B'*rcs_B*eye(3)-rcs_B*rcs_B'); % kg*m^2

%%
% Moment of inertia tensor of the deployable panel about combined center of
% mass projected to the basis vectors of the B frame.
rcd_B = rcb_B - rdb_B;
Jd_C_B = Jd_D_B + md*(rcd_B'*rcd_B*eye(3)-rcd_B*rcd_B'); % kg*m^2

%%
% Combined moment of inertia tensor about the combined center of mass
% projected to the basis vectors of the B frame.
J_C_B = Js_C_B + Jd_C_B; % kg*m^2 

%%
% Combined moment of intertia tensor about the origin of the B frame
% projected to the basis vectors of the B frame. Just for practice.
rbc_B = -rcb_B;
Jc_B_B = J_C_B + (ms+md)*(rbc_B'*rbc_B*eye(3)-rbc_B*rbc_B'); % kg*m^2


%% Principal Moment of Inertia Tensor
% Calculate the direction cosine matrix that projects a vector from the B
% frame to the principal frame of the combined moment of inertia tensor
% about the combined center of mass.
[V, D] = eig(J_C_B);
e1_B = V(:,1);
e1_B = e1_B/norm(e1_B);
e2_B = V(:,2);
e2_B = e2_B/norm(e2_B);
e3_B = cross(e1_B, e2_B);
e3_B = e3_B/norm(e3_B);
A_PB = [e1_B'; e2_B'; e3_B'];
A_BP = A_PB';
%% Addpath to Attitude Representations Folder
addpath('../01 Attitude Representations')
q_PB = A2q(A_PB);
q_BP = A2q(A_BP);

%%
% Combined moment of inertia tensor about the combined center of mass
% projected to the basis vectors of the principal frame. Use A_PB to
% project J_C_B to the principal frame.
J_C_P = A_PB*J_C_B*A_BP; % kg*m^2 (eq 3.66)
