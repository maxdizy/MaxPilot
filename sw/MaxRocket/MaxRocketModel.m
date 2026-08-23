% This file attempts to derive the state space model of North Vector
% Dynamics CM-70 missile based on images from their website

%% environment constants
g_0 = 9.81; % initial acceleration due to gravitym m/s^2
Re = 6371000; % radius of the earth, m

%% environment variables
rho % atmospheric density
P_a % ambient atmospheric pressure
P_ref % reference ambient pressure
g = g_0*(Re^2/(Re+h)^2); % acceleration due to gravity (h = altitude above sea level)

%% missile geometric constants
S % reference area m^2
d % serodynamics reference length of body, m
X_cm % distance from rocket nose to center of mass, m
X_ref % distance from rocket nose to reference moment station, m
m_0 % rocket mass at time 0

%% missile propulsion variables
A_e % rocket nozzle exit area, m^2
F_Pref % reference thrust force, N
I_sp % specific impule of propellant, Ns/Kg

%% missile geometric variables
I_x % inertia on the x axis, Kg*m^2
I_y % inertia on the y axis, Kg*m^2
I_z % inertia on the z axis, Kg*m^2
V_M = sqrt(u^2 + v^2 + w^2); % magnitude of velocity vector of the center of mass of the rocket

%% missile aerodynamic coefficients
C_A % axial force coefficient on Yb axis
C_N % normal force coefficient
C_Ny = C_N * (-v)/(sqrt(v^2 + w^2)); % normal force coefficient on Yb axis
C_Nz = C_N * 1/(sqrt(v^2 + w^2)); % normal force coefficient on Zb axis

C_l = C_ldelta*delta_r + (d/(2*V_M))*C_lp*p; % roll moment coefficient

C_ldelta % slope of curve formed by C_l vs delta (control surface deflection), rad^-1
delta_r % effective control surface deflection causing rolling moment, rad
C_lp % roll damping derivative relative to roll rate p, rad^-1

C_m = C_mref - C_Nz*((X_cm - X_ref)/d) + (d/(2*V_M))*(C_mq + C_ma_dot)*q; % pitch moment coefficient

C_mq % pitch damping derivative relative to pitch rate q_dot, rad_-1
C_ma_dot % pitch damping derivative relative to AoA rate a_dot (slope of C_a vs a), rad_-1

C_n = C_nref + C_Ny*((X_cm - X_ref)/d) + (d/(2*V_M))*(C_nr + C_nb_dot)*r; % yaw moment coefficient

C_nr % yaw damping derivative relative to yaw rate r_dot, rad_-1
C_nb_dot % pitch damping derivative relative to AoS rate b_dot (slope of C_b vs b), rad_-1

% considering a fin deflection robot
C_mref = C_ma*a + C_mdelta*delta_eta;
C_nref = C_nb*b + C_ndelta*delta_xi;

C_ma % slope of the curve formed by C_m vs AoA a, rad_1
C_mdelta % slope of the curve formed by C_m vs delta for pitch, rad_1
C_nb % slope of the curve formed by C_m vs AoS b, rad_1
C_ndelta % slope of the curve formed by C_m vs delta for yaw, rad_1

%% translational forces:
% aerodynamic forces are:
F_Axb = (-1)*0.5*rho*V_M^2*C_A*S;
F_Ayb = 0.5*rho*V_M^2*C_Ny*S;
F_Azb = 0.5*rho*V_M^2*C_A\Nz*S;

% propulsive forces are:
F_p = F_Pref + (P_ref - P_a)*A_e; % fin controlled rocket so no thrust vectoring
F_Pxb = F_p;
F_Pyb = 0;
F_Pzb = 0;

% gravitational forces are:
F_Gxe = 0;
F_Gye = 0;
F_Gze = m*g;

% transform from earth frame to body frame with:
T_eb = [cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi) - cos(theta)*sin(psi), cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);
        cos(theta)*sin(psi), sin(phi)*sin(theta)*cos(psi) + cos(theta)*cos(psi), cos(phi)*sin(theta)*cos(psi) - sin(phi)*sin(psi);
        -sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta)];

F_Gb = T_eb * [F_Gxe; F_Gye; F_Gze];

F_Gxb = F_Gb(1);
F_Gyb = F_Gb(2);
F_Gzb = F_Gb(3);

V_earth = T_eb' * [u; v; w];
x_e = V_earth(1);
y_e = V_earth(2);
z_e = V_earth(3);
h = -z_e;   % if z_e is down-positive (NED convention)

% instantaneous mass is:
m = m_0 - (1/I_sp)*integral(F_Pref, 0, t);

%% rotational moments and angles
% moments in roll, pitch and yaw are:
L_a = 0.5*rho*V_M^2*C_l*S*d; % moment in roll
M_a = 0.5*rho*V_M^2*C_m*S*d; % moment in pith
N_a = 0.5*rho*V_M^2*C_n*S*d; % moment in yaw

% propulsive forces in roll, pitch and yaw are:
L_p = 0; % propulsive force in roll
M_p = 0; % propulsive force in pith
N_p = 0; % propulsive force in yaw

% Angle of Attack (a) and Angle of Sideslip (b) are:
a = arctan(w/u);
b = arcsin(v/V_M);

%% equations of motion
% translational
u_dot = (F_Axb + F_Pxb + F_Gxb)/m - (q*w - r*v);
v_dot = (F_Ayb + F_Pyb + F_Gyb)/m - (r*u - p*w);
w_dot = (F_Azb + F_Pzb + F_Gzb)/m - (p*v - q*u);

% rotational
P_dot = (L_a + L_p - q*r*(I_z - I_y))/I_x;
q_dot = (M_a + M_p - r*p*(I_x - I_z))/I_y;
r_dot = (N_a + N_p - p*q*(I_y - I_z))/I_z; 

% Euler angle equations of motion
phi_dot = p + (q*sin(phi) + r*cos(phi))*tan(theta);
theta_dot = q*cos(phi) - r*sin(phi);
psi_dot = (q*sin(phi) + r*cos(phi))/cos(theta);