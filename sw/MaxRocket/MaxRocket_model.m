% This file attempts to derive the state space model of North Vector
% Dynamics CM-70 missile based on images from their website

function [rho, P_a, P_ref, g, speed_of_sound] = atmosphere(alt)
    % Standard constants
    T_0 = 288.15; % sea-level standard temperature, K
    P_ref = 101325; % sea-level standard pressure, Pa
    L  = 0.0065; % tropospheric lapse rate, K/m
    R  = 287.05; % specific gas constant for air, J/(kg*K)
    Re = 6371000; % redius of the earth, m
    g_0 = 9.80665; % standard gravity, m/s^2
    g = g_0*(Re^2/(Re+alt)^2); % acceleration due to gravity (h = altitude above sea level)
    gamma = 1.4; % ratio of specific heats for air

    if h < 11000
        % Troposphere: linear temperature lapse
        T = T0 - L*alt;
        P_a = P_ref * (T/T_0)^(g_0/(R*L));
    else
        % Tropopause (11 km - 20 km): isothermal layer
        T_11 = T_0 - L*11000; % temperature at 11 km (216.65 K)
        P_11 = P_ref * (T_11/T_0)^(g_0/(R*L)); % pressure at 11 km
        T = T_11; % constant through this layer
        P_a = P_11 * exp(-g_0*(alt - 11000)/(R*T_11));
    end
    rho = P_a / (R*T);
    speed_of_sound = sqrt(gamma*R*T);
end

function [F_Pref, m, d, S, X_cm, I_x, I_y, I_z, C_A, C_N, C_m, C_n, C_l, C_lp, C_mq, C_nr] = burn(t, mach, AoA)
    % for the Cesaroni 614I100-17A motor

    persistent time_points thrust_forces mass_total reference_length reference_area CG_points I_rot I_long C_Axial C_Normal C_Pitch C_Yaw C_Roll C_RollDamp C_PitchDamp
    if isempty(time_points)
        filename = 'MaxRocket_OpenRocket_analysis.csv';
        
        opts = detectImportOptions(filename, 'VariableNamingRule', 'preserve');
        data_table = readtable(filename, opts);
        
        time_points = data_table.('# Time (s)');
        thrust_forces = data_table.('Thrust (N)');
        mass_total = data_table.('Mass (kg)');
        reference_length = data_table.('Reference length (m)');
        reference_area = data_table.('Reference area (mÂ²)');
        CG_points = data_table.('CG location (m)');
        
        I_rot = data_table.('Rotational moment of inertia (kg·m²)');
        I_long = data_table.('Longitudinal moment of inertia (kg·m²)');
        
        C_Axial = fillmissing(data_table.('Axial drag coefficient (​)'), 'constant', 0);
        C_Normal = fillmissing(data_table.('Normal force coefficient (​)'), 'constant', 0);
        C_Pitch = fillmissing(data_table.('Pitch moment coefficient (​)'), 'constant', 0);
        C_Yaw = fillmissing(data_table.('Yaw moment coefficient (​)'), 'constant', 0);
        C_Roll = fillmissing(data_table.('Roll moment coefficient (​)'), 'constant', 0);
        C_RollDamp = fillmissing(data_table.('Roll damping coefficient (​)'), 'constant', 0);
        C_PitchDamp = fillmissing(data_table.('Pitch damping coefficient (​)'), 'constant', 0);
    end

    if t < time_points(1)
        F_Pref = thrust_forces(1);
        m = mass_total(1);
        d = reference_length(1);
        S = reference_area(1);
        X_cm = CG_points(1);
        I_x = I_rot(1);
        I_y = I_long(1);
        I_z = I_long(1);
        C_A = C_Axial(1);
        C_N = C_Normal(1);
        C_m = C_Pitch(1);
        C_n = C_Yaw(1);
        C_l = C_Roll(1);
        C_lp = C_RollDamp(1);
        C_mq = C_PitchDamp(1);
        C_nr = C_PitchDamp(1); % Using pitch damping derivative for yaw damping derivative

    elseif t <= time_points(end)
        F_Pref = interp1(time_points, thrust_forces, t, 'linear'); % reference thrust force, N
        m = interp1(time_points, mass_total, t, 'linear'); % rocket mass, Kg
        d = interp1(time_points, reference_length, t, 'linear'); % aerodynamic reference length of body, m
        S = interp1(time_points, reference_area, t, 'linear'); % reference area m^2
        X_cm = interp1(time_points, CG_points, t, 'linear'); %  instantaneous distance from rocket nose to center of mass, m
        I_x = interp1(time_points, I_rot, t, 'linear'); % inertia on the x axis, Kg*m^2
        I_y = interp1(time_points, I_long, t, 'linear'); % inertia on the y axis, Kg*m^2
        I_z = I_y; % inertia on the z axis, Kg*m^2
        C_A = interp1(time_points, C_Axial, t, 'linear'); % axial force coefficient on Yb axis
        C_N = interp1(time_points, C_Normal, t, 'linear'); % normal force coefficient
        C_m = interp1(time_points, C_Pitch, t, 'linear');
        C_n = interp1(time_points, C_Yaw, t, 'linear');
        C_l = interp1(time_points, C_Roll, t, 'linear');
        C_lp = interp1(time_points, C_RollDamp, t, 'linear'); % roll damping derivative relative to roll rate p, rad^-1
        C_mq = interp1(time_points, C_PitchDamp, t, 'linear'); % pitch damping derivative relative to pitch rate q_dot, rad_-1
        C_nr = C_mq; % yaw damping derivative relative to yaw rate r_dot, rad_-1

    else
        F_Pref = thrust_forces(end);
        m = mass_total(end);
        X_cm = CG_points(end);
        I_x = I_rot(end);
        I_y = I_long(end);
        I_z = I_long(end);
        C_A = C_Axial(end);
        C_N = C_Normal(end);
        C_m = C_Pitch(end);
        C_n = C_Yaw(end);
        C_l = C_Roll(end);
        C_lp = C_RollDamp(end);
        C_mq = C_PitchDamp(end);
        C_nr = C_PitchDamp(end);
    end
end

function [u_dot, v_dot, w_dot, p_dot, q_dot, r_dot, phi_dot, theta_dot, psi_dot] = rocket_physics(x, u_ctrl, t)
    %% missile variables
    u = x(1); v = x(2); w = x(3); p = x(4); q = x(5); r = x(6); phi = x(7); theta = x(8); psi = x(9); alt = x(10);
    delta_eta = u_ctrl(1); delta_xi = u_ctrl(2);
    [rho, P_a, P_ref, g, speed_of_sound] = atmosphere(alt);
    V_M = sqrt(u^2 + v^2 + w^2); % magnitude of velocity vector of the center of mass of the rocket
    mach = V_M/speed_of_sound; % mach number
    AoA = arctan(w/u); % angle of attack
    AoS = arcsin(v/V_M); % angle of sideslip
    X_ref = 0; % distance from rocket nose to reference moment station, m
    A_e = 0; % rocket nozzle exit area, m^2 (0 b/c pressure-thrust variance is negligible for this model)
    
    [F_Pref, m, d, S, X_cm, I_x, I_y, I_z, C_A, C_N, ~, ~, ~, C_lp, C_mq, C_nr] = burn(t, mach, AoA);
    [C_leta, C_lxi, C_meta, C_mxi, C_neta, C_nxi, C_ma, C_nb] = find_active_control_variables(mach);

    %% missile aerodynamic coefficients
    % find roll moment coefficient
    % C_ldelta % slope of curve formed by C_l vs delta (control surface deflection), rad^-1
    % C_l = C_ldelta*delta_r + (d/(2*V_M))*C_lp*p; % roll moment coefficient
    C_l = C_leta*delta_eta + C_lxi*delta_xi + (d/(2*V_M))*C_lp*p; % roll moment coefficient

    % find pitch moment coefficient
    % C_ma % slope of the curve formed by C_m vs AoA a, rad_1
    % C_mdelta % slope of the curve formed by C_m vs delta for pitch, rad_1
    % C_mref = C_ma*AoA + C_mdelta*delta_eta; % considering a fin deflection rocket
    C_mref = C_ma*AoA + C_meta*delta_eta + C_mxi*delta_xi; % considering a fin deflection rocket
    C_Nz = C_N * 1/(sqrt(v^2 + w^2)); % normal force coefficient on Zb axis
    C_ma_dot = 0; % pitch damping derivative relative to AoA rate a_dot (slope of C_a vs a), rad_-1 (assuming 0 b/c negligible for this model)
    C_m = C_mref - C_Nz*((X_cm - X_ref)/d) + (d/(2*V_M))*(C_mq + C_ma_dot)*q; % pitch moment coefficient
    
    % find yaw moment coefficient
    % C_nb  % slope of the curve formed by C_m vs AoS b, rad_1
    % C_ndelta % slope of the curve formed by C_m vs delta for yaw, rad_1
    % C_nref = C_nb*AoS + C_ndelta*delta_xi; % considering a fin deflection rocket
    C_nref = C_nb*AoS + C_neta*delta_eta + C_nxi*delta_xi; % considering a fin deflection rocket
    C_Ny = C_N * (-v)/(sqrt(v^2 + w^2)); % normal force coefficient on Yb axis
    C_nb_dot = 0; % pitch damping derivative relative to AoS rate b_dot (slope of C_b vs b), rad_-1 (assuming 0 b/c negligible for this model)
    C_n = C_nref + C_Ny*((X_cm - X_ref)/d) + (d/(2*V_M))*(C_nr + C_nb_dot)*r; % yaw moment coefficient  

    % transform from earth frame to body frame with:
    T_eb = [cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi) - cos(theta)*sin(psi), cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);
            cos(theta)*sin(psi), sin(phi)*sin(theta)*cos(psi) + cos(theta)*cos(psi), cos(phi)*sin(theta)*cos(psi) - sin(phi)*sin(psi);
            -sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta)];
    
    %% translational forces: 
    % aerodynamic forces are:
    F_Axb = (-1)*0.5*rho*V_M^2*C_A*S;
    F_Ayb = 0.5*rho*V_M^2*C_Ny*S;
    F_Azb = 0.5*rho*V_M^2*C_A\C_Nz*S;
    
    % propulsive forces are:
    F_p = F_Pref + (P_ref - P_a)*A_e; % fin controlled rocket so no thrust vectoring
    F_Pxb = F_p;
    F_Pyb = 0;
    F_Pzb = 0;

    % gravitational forces are:
    F_Gxe = 0;
    F_Gye = 0;
    F_Gze = m*g;
    
    F_Gb = T_eb * [F_Gxe; F_Gye; F_Gze];
    
    F_Gxb = F_Gb(1);
    F_Gyb = F_Gb(2);
    F_Gzb = F_Gb(3);

    %% rotational moments and angles
    % moments in roll, pitch and yaw are:
    L_a = 0.5*rho*V_M^2*C_l*S*d; % moment in roll
    M_a = 0.5*rho*V_M^2*C_m*S*d; % moment in pith
    N_a = 0.5*rho*V_M^2*C_n*S*d; % moment in yaw
    
    % propulsive forces in roll, pitch and yaw are:
    L_p = 0; % propulsive force in roll
    M_p = 0; % propulsive force in pith
    N_p = 0; % propulsive force in yaw
    
    %% equations of motion
    % translational
    u_dot = (F_Axb + F_Pxb + F_Gxb)/m - (q*w - r*v);
    v_dot = (F_Ayb + F_Pyb + F_Gyb)/m - (r*u - p*w);
    w_dot = (F_Azb + F_Pzb + F_Gzb)/m - (p*v - q*u);
    
    % rotational
    p_dot = (L_a + L_p - q*r*(I_z - I_y))/I_x;
    q_dot = (M_a + M_p - r*p*(I_x - I_z))/I_y;
    r_dot = (N_a + N_p - p*q*(I_y - I_z))/I_z; 
    
    % Euler angle equations of motion
    phi_dot = p + (q*sin(phi) + r*cos(phi))*tan(theta);
    theta_dot = q*cos(phi) - r*sin(phi);
    psi_dot = (q*sin(phi) + r*cos(phi))/cos(theta);
end
