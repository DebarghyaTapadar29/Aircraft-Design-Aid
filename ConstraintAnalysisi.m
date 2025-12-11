%% AIRCRAFT CONSTRAINT ANALYSIS PROGRAM (SINGLE-FILE VERSION)
%
% This program calculates the required Sea Level Static Thrust-to-Weight Ratio
%
% This version has been updated with a corrected landing ground roll formula
% and now generates both summary tables and a plot for clear visualization.

clear; clc; close all;

disp('--- Constraint Analysis for the Advanced Strategic Penetrator (ASP-A) ---');

%% USER INPUT DATA
% --- Data derived from all provided documentation ---

% -- Aircraft Mass and Geometry --
W_TO_mass = 155000;      % Maximum Take-Off Weight (kg) [Source: ASP Doc, p.6]
W_fuel_total = 50000;    % Max Fuel Weight (kg) [Source: ASP Doc, p.6]
W_Land_mass = W_TO_mass - 0.90 * W_fuel_total; % Assume 90% fuel burn for landing
W_man_mass = W_TO_mass - 0.50 * W_fuel_total;  % Assume 50% fuel burn for maneuver

% Geometric data from "Drag Analysis" doc (Supersonic Config, M >= 1.0)
S_ref = 460;            % Total Lifting Reference Area (m^2)
AR = 2.3;               % Wing Aspect Ratio
Ne = 4;                 % Number of Engines

% -- Aerodynamics (for Mach-Dependent Calculations) --
S_wet = 760;           % Aircraft Wetted Area (m^2)
C_fe = 0.0025;          % Equivalent Skin Friction Coefficient
C_L_min_drag = 0.0;     % Lift Coefficient at Minimum Drag. Assumed 0.
sweep_qtr_chord_deg = 46; % Quarter-Chord Sweep (degrees), supersonic config
tc_ratio = 0.06;        % Wing Thickness-to-Chord Ratio
dihedral_deg = -2;       % Wing dihedral (degrees) - Assumed 0

% Lift Coefficients (Assumed typical values)
C_Lmax_TO = 4.3;        % Max lift coefficient for takeoff config
C_Lmax_Land = 4;      % Max lift coefficient for landing config

% -- Performance Requirements (Examples - can be adjusted) --
n_req = 2.5;            % Required Load Factor for sustained turn (e.g., 3.5g)
Ps_req = 150;           % Required Specific Excess Power (m/s)
ROC_req = 40;           % Required Rate of Climb (m/s)

% -- Point Performance Requirements (Takeoff & Landing) --
Vstall_TO_req = 65;     % Required Takeoff Stall Speed (m/s, ~126 kts)
Vstall_Land_req = 60;   % Required Landing Stall Speed (m/s, ~117 kts)
SSCG_req = 0.012;       % Second Stage Climb Gradient (1.2% for 4-engine aircraft, OEI)
S_Land_req = 3000;      % Required Landing Ground Roll (m)
TOP = 2500;             % Takeoff Parameter (from BFL analysis)
mu_roll = 0.3;          % Coefficient of rolling friction for landing

%% SETUP & CONSTANTS
g = 9.81;               % Acceleration due to gravity (m/s^2)
W_TO = W_TO_mass * g;   % Max Takeoff Weight (N)
W_S = W_TO / S_ref;     % Wing Loading at MTOW (N/m^2)

[~, ~, rho_sl, ~] = get_isa_properties(0);

% Define Flight Envelope for Parametric Analysis
M_vec = 0.2:0.2:3.2;         % Mach number vector
h_m_vec = 0:50:25000;        % Altitude vector (meters), 0 to 25km (82,000 ft)

% Initialize matrices to store results
Tsl_Wto_results = nan(length(h_m_vec), length(M_vec), 3);
constraint_names = ["Sustained Turn", "Specific Excess Power", "Rate of Climb"];

%% MAIN CALCULATION LOOP
disp('Calculating performance across the flight envelope...');
fprintf('Using specified aircraft W/S = %.2f N/m^2\n', W_S);

CD_min_profile = C_fe * S_wet / S_ref;
sweep_rad = deg2rad(sweep_qtr_chord_deg);
dihedral_rad = deg2rad(dihedral_deg);

for i = 1:length(h_m_vec)
    h = h_m_vec(i);
    [~, ~, rho, a_sound] = get_isa_properties(h);
    
    for j = 1:length(M_vec)
        M = M_vec(j);
        
        V = M * a_sound;
        q = 0.5 * rho * V^2;
        alpha = rho / rho_sl;
        beta_man = (W_man_mass*g) / W_TO;
        
        if q == 0 || alpha == 0; continue; end
        
        fAwing = 0.005 * (1 + 1.5 * (sweep_rad - 0.6)^2);
        term1 = 1 + fAwing * ( (1 + cos(sweep_rad))^2 / tc_ratio^0.3 );
        term2 = 0.005 * Ne^0.5 * (1 + tan(dihedral_rad));
        e = (1 + 0.12 * M^2)^-1 * (term1 + term2);
        k1 = 1 / (pi * AR * e);
        CD0 = CD_min_profile + k1 * C_L_min_drag^2;
        
        % 1. Sustained Turn Constraint
        A_turn = (beta_man * q * CD0) / alpha;
        B_turn = (beta_man * k1 * n_req^2) / (alpha * q);
        C_turn = 0;
        Tsl_Wto_results(i, j, 1) = (A_turn / W_S) + (B_turn * W_S) + C_turn;
        
        % 2. Specific Excess Power (Ps) Constraint
        A_Ps = (beta_man * q * CD0) / alpha;
        B_Ps = (beta_man * k1 * 1^2) / (alpha * q); % n=1 for level flight
        C_Ps = (beta_man * Ps_req) / (alpha * V);
        Tsl_Wto_results(i, j, 2) = (A_Ps / W_S) + (B_Ps * W_S) + C_Ps;
        
        % 3. Rate of Climb (ROC) Constraint
        A_ROC = A_Ps; B_ROC = B_Ps; % Aero terms are the same as for Ps
        C_ROC = (beta_man * ROC_req) / (alpha * V);
        Tsl_Wto_results(i, j, 3) = (A_ROC / W_S) + (B_ROC * W_S) + C_ROC;
    end
end
disp('Calculation complete.');
disp(' ');

%% POINT PERFORMANCE CALCULATIONS
disp('--- Point Performance Constraint Calculations ---');
% Stall Speed Constraint (defines max W/S for a given stall speed)
W_S_max_takeoff = 0.5 * rho_sl * Vstall_TO_req^2 * C_Lmax_TO;
W_S_max_landing = 0.5 * rho_sl * Vstall_Land_req^2 * C_Lmax_Land;
W_S_max_landing_at_TO_weight = W_S_max_landing / (W_Land_mass / W_TO_mass);
fprintf('Stall Speed Constraint (Takeoff): Max W/S = %.2f N/m^2\n', W_S_max_takeoff);
fprintf('Stall Speed Constraint (Landing): Max W/S at takeoff weight = %.2f N/m^2\n', W_S_max_landing_at_TO_weight);

% Climb Gradient Constraint (defines min T/W for OEI climb)
AR_subsonic = 6.0; S_ref_subsonic = 170.0; M_climb = 0.25;
fAwing_sub = 0.005 * (1 + 1.5 * (deg2rad(46) - 0.6)^2);
term1_sub = 1 + fAwing_sub * ( (1 + cos(deg2rad(46)))^2 / tc_ratio^0.3 );
term2_sub = 0.005 * Ne^0.5 * (1 + tan(dihedral_rad));
e_climb = (1 + 0.12 * M_climb^2)^-1 * (term1_sub + term2_sub);
k1_climb = 1 / (pi * AR_subsonic * e_climb);
CD_min_prof_climb = C_fe * S_wet / S_ref_subsonic;
CD0_climb = CD_min_prof_climb + k1_climb * C_L_min_drag^2;
V_climb = 1.2 * Vstall_TO_req; q_climb = 0.5 * rho_sl * V_climb^2;
CL_climb = (W_TO_mass*g) / (q_climb * S_ref_subsonic);
CD_climb = CD0_climb + k1_climb * CL_climb^2;
L_D_climb = CL_climb / CD_climb;
T_W_climb_req = SSCG_req + (1/L_D_climb);
T_all_engines_W_TO = T_W_climb_req * (Ne / (Ne - 1));
Tsl_Wto_climb_req = T_all_engines_W_TO; % Assuming thrust at V_climb is ~100% SL static
fprintf('Climb Gradient Constraint: Required T_sl/W_to >= %.3f\n', Tsl_Wto_climb_req);

% --- FIX APPLIED HERE ---
% Landing Ground Roll Constraint (defines max W/S for a given landing distance)
% The original formula was dimensionally incorrect. This is a corrected version
% derived from S_g ≈ (1.15 * V_stall)^2 / (2 * g * mu_roll).
WS_at_landing = (S_Land_req * rho_sl * g * mu_roll * C_Lmax_Land) / 1.3225;
W_S_land_ground_roll = WS_at_landing / (W_Land_mass / W_TO_mass);
fprintf('Landing Ground Roll Constraint: Max W/S at takeoff weight = %.2f N/m^2\n', W_S_land_ground_roll);

% Balanced Field Length Constraint (defines min T/W for a given takeoff distance)
sigma_TO = 1.0;
Tsl_Wto_BFL_req = W_S / (sigma_TO * C_Lmax_TO * TOP);
fprintf('Balanced Field Length Constraint: Required T_sl/W_to >= %.3f\n', Tsl_Wto_BFL_req);
disp('-----------------------------------------------');

%% DISPLAY RESULTS - TABLES FOR TYPICAL ALTITUDES
disp(' ');
disp('--- T_sl/W_to Requirements at Typical Altitudes ---');

alt_to_display = [0, 5000, 10000, 15000, 20000];
colNames = "M_" + strrep(string(M_vec), '.', '_');
rowNames = "Alt_" + string(alt_to_display') + "_m";

for k = 1:length(constraint_names)
    table_data = zeros(length(alt_to_display), length(M_vec));
    for a = 1:length(alt_to_display)
        h_disp = alt_to_display(a);
        [~, idx] = min(abs(h_m_vec - h_disp));
        table_data(a, :) = Tsl_Wto_results(idx, :, k);
    end
    fprintf('\n====== TABLE FOR CONSTRAINT: %s ======\n', constraint_names(k));
    ResultsTable = array2table(table_data, 'VariableNames', colNames, 'RowNames', rowNames);
    disp(ResultsTable);
end

%% VISUALIZE RESULTS (PLOTTING)
figure;
hold on; grid on;

plot_legends = {};
for h_plot = alt_to_display
    [~, idx] = min(abs(h_m_vec - h_plot));
    Tsl_Wto_at_h = Tsl_Wto_results(idx, :, 1); % Index 1 = Sustained Turn
    plot(M_vec, Tsl_Wto_at_h, 'LineWidth', 2);
    plot_legends{end+1} = sprintf('Altitude = %d m', round(h_m_vec(idx)));
end

title(sprintf('ASP-A Constraint: %s (n = %.1fg)', constraint_names(1), n_req));
xlabel('Mach Number (M)');
ylabel('Required Thrust-to-Weight Ratio (T_{sl}/W_{to})');
legend(plot_legends, 'Location', 'northwest');
ylim([0, 1.5]);
hold off;

disp(' ');
disp('Plot generated successfully.');

%% ========================================================================
%  HELPER FUNCTION FOR ATMOSPHERE MODEL
%  ========================================================================
function [T, P, rho, a] = get_isa_properties(h)
% GET_ISA_PROPERTIES Calculates atmospheric properties based on the 
% International Standard Atmosphere (ISA) model.

% ISA constants
T0 = 288.15;       % Sea level temperature (K)
P0 = 101325;       % Sea level pressure (Pa)
R = 287.058;       % Specific gas constant for dry air (J/(kg·K))
g = 9.80665;       % Gravitational acceleration (m/s^2)
gamma = 1.4;       % Ratio of specific heats for air
L = -0.0065;       % Temperature lapse rate in troposphere (K/m)

h_tropo = 11000;   % Tropopause altitude (m)
h_strato1 = 20000; % Stratosphere layer 1 limit (m)

if h <= h_tropo % Troposphere
    T = T0 + L * h;
    P = P0 * (T / T0)^(-g / (L * R));
elseif h > h_tropo && h <= h_strato1 % Lower Stratosphere
    T_tropo = T0 + L * h_tropo;
    P_tropo = P0 * (T_tropo / T0)^(-g / (L * R));
    T = T_tropo;
    P = P_tropo * exp(-g * (h - h_tropo) / (R * T));
else % Upper Stratosphere
    T_tropo = T0 + L * h_tropo;
    P_tropo = P0 * (T_tropo / T0)^(-g / (L * R));
    T_strato1 = T_tropo;
    P_strato1 = P_tropo * exp(-g * (h_strato1 - h_tropo) / (R * T_strato1));
    L_strato = 0.001;
    T = T_strato1 + L_strato * (h - h_strato1);
    P = P_strato1 * (T / T_strato1)^(-g / (L_strato * R));
end

rho = P / (R * T);
a = sqrt(gamma * R * T);
end
