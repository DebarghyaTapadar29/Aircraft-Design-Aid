% =========================================================================
%      MILITARY AIRCRAFT PRELIMINARY ANALYSIS TOOL 
% =========================================================================
%
% DESCRIPTION:
% This is a self-contained MATLAB script for the preliminary analysis of
% military aircraft. It allows the user to select an aircraft archetype
% (Fighter, Bomber, Transport) based on US and Soviet/Russian design
% philosophies. The tool then calculates aerodynamic performance across a
% wide flight envelope, generates 3D plots, creates a constraint analysis
% diagram, and produces a detailed design report with a scoring matrix.
%
% =========================================================================

clear; clc; close all;

%% 1. USER SELECTION OF AIRCRAFT TYPE
fprintf('-----------------------------------------------------\n');
fprintf(' Military Aircraft Preliminary Analysis Tool (Verbose)\n');
fprintf('-----------------------------------------------------\n');
disp('Select an aircraft archetype to analyze:');
disp('  [1] US-Style Agile Fighter (e.g., F-16/F-18 Philosophy)');
disp('  [2] Soviet-Style Maneuverability Fighter (e.g., Su-27/MiG-29 Philosophy)');
disp('  [3] Strategic Bomber (e.g., B-1/Tu-160 Philosophy)');
disp('  [4] Military Transport (e.g., C-17/Il-76 Philosophy)');

choice = input('\nEnter your choice [1-4]: ');

archetype_map = {'us_fighter', 'soviet_fighter', 'strategic_bomber', 'transport'};
if isempty(choice) || ~ismember(choice, 1:4)
    error('Invalid selection. Please run the script again and choose a number between 1 and 4.');
end
selected_archetype = archetype_map{choice};

% Load the corresponding aircraft data structure using a local function
AC = get_aircraft_data(selected_archetype);

%% 2. DEFINE ANALYSIS FLIGHT ENVELOPE
mach_vector = 0.1:0.1:3.0;
alt_vector_km = 0.1:0.5:25;
alt_vector_m = alt_vector_km * 1000;

fprintf('\n--- Starting Analysis for: %s ---\n', AC.name);
fprintf('Iterating over %d Mach points and %d altitude points...\n', ...
        length(mach_vector), length(alt_vector_m));
fprintf('NOTE: Detailed loop calculations will be printed for two representative points only.\n');

%% 3. PRE-ALLOCATE & RUN CORE ANALYSIS LOOP
num_mach = length(mach_vector);
num_alt = length(alt_vector_m);

Results.CD0 = zeros(num_mach, num_alt);
Results.K = zeros(num_mach, num_alt);
Results.CL_max = zeros(num_mach, num_alt);
Results.LD_max = zeros(num_mach, num_alt);
Results.Thrust_avail_max = zeros(num_mach, num_alt);
Results.Thrust_avail_mil = zeros(num_mach, num_alt);

% Main analysis loop
for i = 1:num_mach
    for j = 1:num_alt
        mach = mach_vector(i);
        alt = alt_vector_m(j);
        
        % --- Verbose Flag: Print details for first point and a supersonic point ---
        print_this_iteration = (i == 1 && j == 1) || (i == 20 && j == 31); % M=2.0 at 15100m
        
        if print_this_iteration
           fprintf('\n========================================================================\n');
           fprintf('>>> DETAILED LOOP CALCULATION (Point: M=%.1f, Alt=%.0f m) <<<\n', mach, alt);
           fprintf('========================================================================\n');
        end
        
        Atmo = isa_model(alt, print_this_iteration);
        Aero = calculate_aerodynamics(AC, mach, Atmo, print_this_iteration);
        
        Results.CD0(i, j) = Aero.CD0;
        Results.K(i, j) = Aero.K;
        Results.CL_max(i, j) = Aero.CL_max;
        
        if Aero.CD0 > 0 && Aero.K > 0
            ld_max_val = sqrt(1 / (4 * Aero.CD0 * Aero.K));
            Results.LD_max(i, j) = ld_max_val;
            if print_this_iteration
                fprintf('>> CALC (Main Loop): Max L/D Ratio\n');
                fprintf('   LD_max = sqrt(1 / (4 * CD0 * K)) = sqrt(1 / (4 * %.4f * %.4f)) = %.3f\n', Aero.CD0, Aero.K, ld_max_val);
            end
        end
        
        [T_max, T_mil] = AC.Prop.ThrustFn(mach, Atmo, print_this_iteration);
        Results.Thrust_avail_max(i, j) = T_max;
        Results.Thrust_avail_mil(i, j) = T_mil;
    end
end
fprintf('\n...Analysis loop complete.\n');

%% 4. GENERATE PLOTS, SCORES, AND REPORT
fprintf('Generating outputs...\n\n');
generate_outputs(AC, mach_vector, alt_vector_km, Results, []); 

fprintf('--- Analysis Finished ---\n');

% --- End of Main Script ---


%% --------------------------------------------------------------------- %%
%                        >>> LOCAL FUNCTIONS <<<                          %
%  All supporting functions are defined below this point. The main script %
%  ends above. Every function must have a corresponding 'end' statement.  %
%% --------------------------------------------------------------------- %%


% =========================================================================
% FUNCTION 1: AIRCRAFT DATABASE
% =========================================================================
function AC = get_aircraft_data(archetype)
    fprintf('\n--- CONFIGURING AIRCRAFT DATABASE ---\n');
    switch archetype
        case 'us_fighter'
            fprintf('Selected: US-Style Agile Fighter\n');
            AC.name = 'US-Style Agile Fighter';
            AC.Wing.S_ref = 35; AC.Wing.AR = 3.2; AC.Wing.Lambda_c4_deg = 35; AC.Wing.taper = 0.25; AC.Wing.tc_avg = 0.045;
            AC.Weight.TOW_guess = 16000; AC.Aero.CD0_subsonic_base = 0.018;
            AC.Prop.num_engines = 1; AC.Prop.engine_class = 'high_bypass_ab';
            AC.Req.V_stall_max_kts = 125; AC.Req.Takeoff_dist_m = 800; AC.Req.Sustained_Turn_g = 7; AC.Req.Supercruise_mach = 1.2;
        case 'soviet_fighter'
            fprintf('Selected: Soviet-Style Maneuverability Fighter\n');
            AC.name = 'Soviet-Style Maneuverability Fighter';
            AC.Wing.S_ref = 62; AC.Wing.AR = 2.8; AC.Wing.Lambda_c4_deg = 42; AC.Wing.taper = 0.15; AC.Wing.tc_avg = 0.05;
            AC.Weight.TOW_guess = 24000; AC.Aero.CD0_subsonic_base = 0.022;
            AC.Prop.num_engines = 2; AC.Prop.engine_class = 'low_bypass_ab';
            AC.Req.V_stall_max_kts = 115; AC.Req.Takeoff_dist_m = 650; AC.Req.Sustained_Turn_g = 6.5; AC.Req.Supercruise_mach = 1.4;
        case 'strategic_bomber'
            fprintf('Selected: Strategic Bomber\n');
            AC.name = 'Strategic Bomber';
            AC.Wing.S_ref = 160; AC.Wing.AR = 4.5; AC.Wing.Lambda_c4_deg = 60; AC.Wing.taper = 0.2; AC.Wing.tc_avg = 0.06;
            AC.Weight.TOW_guess = 90000; AC.Aero.CD0_subsonic_base = 0.014;
            AC.Prop.num_engines = 2; AC.Prop.engine_class = 'low_bypass_ab';
            AC.Req.V_stall_max_kts = 140; AC.Req.Takeoff_dist_m = 2500; AC.Req.Sustained_Turn_g = 1.5; AC.Req.Supercruise_mach = 1.2;
        case 'transport'
            fprintf('Selected: Military Transport\n');
            AC.name = 'Military Transport';
            AC.Wing.S_ref = 350; AC.Wing.AR = 8.0; AC.Wing.Lambda_c4_deg = 25; AC.Wing.taper = 0.3; AC.Wing.tc_avg = 0.12;
            AC.Weight.TOW_guess = 265000; AC.Aero.CD0_subsonic_base = 0.025;
            AC.Prop.num_engines = 4; AC.Prop.engine_class = 'high_bypass_dry';
            AC.Req.V_stall_max_kts = 100; AC.Req.Takeoff_dist_m = 900; AC.Req.Sustained_Turn_g = 1.2; AC.Req.Supercruise_mach = 0;
    end
    
    AC.Wing.b = sqrt(AC.Wing.AR * AC.Wing.S_ref);
    fprintf('>> CALC (get_aircraft_data): Calculating wingspan (b)...\n');
    fprintf('   b = sqrt(AR * S_ref) = sqrt(%.2f * %.2f) = %.3f m\n', AC.Wing.AR, AC.Wing.S_ref, AC.Wing.b);

    if contains(archetype, 'fighter'), AC.Wing.cl_max_base = 1.4; AC.HighLift.delta_CL_max_flap = 0.7; AC.HighLift.delta_CL_max_slat = 0.4;
    else, AC.Wing.cl_max_base = 1.2; AC.HighLift.delta_CL_max_flap = 1.2; AC.HighLift.delta_CL_max_slat = 0.6; end
    
    AC.Prop.ThrustFn = @(m, atmo, verb) get_engine_thrust(AC.Prop.engine_class, m, atmo, AC.Prop.num_engines, verb);
end


% =========================================================================
% FUNCTION 2: OUTPUT AND REPORT GENERATOR
% =========================================================================
function generate_outputs(AC, mach_vector, alt_vector_km, Results, ~)
    fprintf('\n\n--- DETAILED REPORT & CONSTRAINT CALCULATIONS ---\n');
    g = 9.81;
    if contains(AC.name, 'Transport'), WS_vec = 1000:100:8000;
    elseif contains(AC.name, 'Bomber'), WS_vec = 2000:100:10000;
    else, WS_vec = 1500:100:7000; end
        
    % --- Stall Constraint Calculations ---
    fprintf('\n[1] Stall Speed Constraint Calculation\n');
    rho_sl_atmo = isa_model(0, false);
    rho_sl = rho_sl_atmo.rho;
    V_stall_mps = AC.Req.V_stall_max_kts * 0.5144;
    fprintf('   - Required Stall Speed: %.1f kts = %.2f m/s\n', AC.Req.V_stall_max_kts, V_stall_mps);
    Aero_land = calculate_aerodynamics(AC, 0.2, rho_sl_atmo, false);
    fprintf('   - Landing CL_max (at M=0.2): %.4f\n', Aero_land.CL_max);
    CData.Stall.WS_max = 0.5 * rho_sl * V_stall_mps^2 * Aero_land.CL_max;
    fprintf('   - Max Wing Loading (W/S) = 0.5 * rho_sl * V_stall^2 * CL_max = 0.5 * %.3f * %.2f^2 * %.4f = %.1f N/m^2\n', rho_sl, V_stall_mps, Aero_land.CL_max, CData.Stall.WS_max);
    
    % --- Takeoff Constraint Calculations ---
    fprintf('\n[2] Takeoff Distance Constraint Calculation\n');
    fprintf('   - Required Takeoff Distance: %d m\n', AC.Req.Takeoff_dist_m);
    fprintf('   - Takeoff T/W calculated using formula: T/W = (1/(0.7*g*Dist)) * (W/S) / (rho_sl*(CL_max_land/1.21))\n');
    CData.Takeoff.TW = (1/(0.7*g*AC.Req.Takeoff_dist_m)) .* WS_vec ./ (rho_sl*(Aero_land.CL_max/1.21));

    % --- Sustained Turn Constraint Calculations ---
    fprintf('\n[3] Sustained Turn Constraint Calculation\n');
    Aero_turn = calculate_aerodynamics(AC, 0.8, isa_model(5000,false), false);
    atmo_turn = isa_model(5000, false);
    q_turn = 0.5 * atmo_turn.rho * (0.8 * atmo_turn.a)^2;
    fprintf('   - Turn Condition: M=0.8 at 5000m, n = %.1f g\n', AC.Req.Sustained_Turn_g);
    fprintf('   - Dynamic Pressure (q) at turn condition: %.1f Pa\n', q_turn);
    fprintf('   - Turn T/W calculated using formula: T/W = q*CD0/(W/S) + (K*n^2/q)*(W/S)\n');
    CData.Turn.TW = q_turn*Aero_turn.CD0./WS_vec + (Aero_turn.K*AC.Req.Sustained_Turn_g^2/q_turn).*WS_vec;
    
    % --- Supercruise Constraint Calculations ---
    if AC.Req.Supercruise_mach > 0
        fprintf('\n[4] Supercruise Constraint Calculation\n');
        Aero_sc = calculate_aerodynamics(AC, AC.Req.Supercruise_mach, isa_model(12000,false), false);
        atmo_sc = isa_model(12000, false);
        q_sc = 0.5*atmo_sc.rho*(AC.Req.Supercruise_mach*atmo_sc.a)^2;
        fprintf('   - Supercruise Condition: M=%.1f at 12000m, n = 1g (level flight)\n', AC.Req.Supercruise_mach);
        fprintf('   - Dynamic Pressure (q) at supercruise condition: %.1f Pa\n', q_sc);
        fprintf('   - Supercruise T/W calculated using formula: T/W = q*CD0/(W/S) + (K/q)*(W/S)\n');
        CData.Supercruise.TW = q_sc*Aero_sc.CD0./WS_vec + (Aero_sc.K/q_sc).*WS_vec;
    end
    CData.WS_vector = WS_vec / g;
    
    % --- Generate Plots (no calcs here) ---
    [M, A] = meshgrid(mach_vector, alt_vector_km);
    figure('Name', [AC.name, ' - Performance Plots'], 'NumberTitle', 'off');
    subplot(2,2,1); surf(M, A, Results.CD0'); shading interp; title('Zero-Lift Drag (CD0)'); xlabel('Mach'); ylabel('Alt (km)'); colorbar;
    subplot(2,2,2); surf(M, A, Results.LD_max'); shading interp; title('Max L/D Ratio'); xlabel('Mach'); ylabel('Alt (km)'); colorbar;
    subplot(2,2,3); surf(M, A, Results.Thrust_avail_mil' / 1000); shading interp; title('Military Thrust (kN)'); xlabel('Mach'); ylabel('Alt (km)'); colorbar;
    subplot(2,2,4); surf(M, A, Results.Thrust_avail_max' / 1000); shading interp; title('Max A/B Thrust (kN)'); xlabel('Mach'); ylabel('Alt (km)'); colorbar;
    
    figure('Name', [AC.name, ' - Constraint Analysis'], 'NumberTitle', 'off'); hold on; grid on;
    plot(CData.WS_vector, CData.Turn.TW, 'r-', 'LineWidth', 2); plot(CData.WS_vector, CData.Takeoff.TW, 'b-', 'LineWidth', 2); plot([CData.Stall.WS_max/g, CData.Stall.WS_max/g], [0, 2], 'k--', 'LineWidth', 2);
    legend_items = {'Sustained Turn', 'Takeoff', 'Stall Speed'};
    if AC.Req.Supercruise_mach > 0, plot(CData.WS_vector, CData.Supercruise.TW, 'g-', 'LineWidth', 2); legend_items{4} = 'Supercruise'; end
    legend(legend_items, 'Location', 'northwest'); xlabel('Wing Loading (W/S, kg/m^2)'); ylabel('Thrust-to-Weight Ratio (T/W)'); title(['Constraint Diagram for ', AC.name]);
    
    % --- Design Point Calculation ---
    fprintf('\n[5] Design Point Parameter Calculation\n');
    W0_S = AC.Weight.TOW_guess / AC.Wing.S_ref;
    fprintf('   - Wing Loading (W0/S) = TOW_guess / S_ref = %d kg / %.1f m^2 = %.2f kg/m^2\n', AC.Weight.TOW_guess, AC.Wing.S_ref, W0_S);
    [T0_max, ~] = get_engine_thrust(AC.Prop.engine_class, 0, isa_model(0, false), AC.Prop.num_engines, false);
    fprintf('   - Sea Level Static Max Thrust (T0_max) = %.0f N\n', T0_max);
    T0_W0 = T0_max / (AC.Weight.TOW_guess*g);
    fprintf('   - Thrust-to-Weight (T0/W0) = T0_max / (TOW*g) = %.0f N / (%.0f kg * 9.81) = %.3f\n', T0_max, AC.Weight.TOW_guess, T0_W0);
    plot(W0_S, T0_W0, 'kp', 'MarkerSize', 15, 'MarkerFaceColor', 'yellow', 'DisplayName', 'Design Point');
    axis tight; ylim([0, iif(contains(AC.name, 'Fighter'), 1.6, 0.7)]);
    
    fprintf('\n\n============================================================\n'); fprintf('           DESIGN ASSESSMENT REPORT: %s\n', upper(AC.name)); fprintf('============================================================\n\n');
    fprintf('*** 1. DESIGN PHILOSOPHY & KEY PARAMETERS ***\n'); fprintf('This archetype prioritizes: %s\n', get_philosophy(AC.name)); fprintf('Key Parameters: T/W = %.2f, W/S = %.1f kg/m^2, TOW = %d kg\n\n', T0_W0, W0_S, AC.Weight.TOW_guess);
    
    % --- Performance Scoring Calculations ---
    fprintf('*** 2. PERFORMANCE SCORING MATRIX (Archetype-Adjusted, 1-10) ***\n');
    [target_LD_sub, target_dCD0, target_LD_sup, target_Ps] = get_scoring_targets(AC.name);
    
    fprintf('\n   Metric 1: Subsonic L/D\n');
    actual_LD_sub = max(Results.LD_max(mach_vector < 1 & mach_vector > 0.5), [], 'all');
    score_sub = min(10, max(0, (actual_LD_sub / target_LD_sub) * 10));
    fprintf('     - Actual = max L/D for M in [0.6, 0.9] = %.2f\n', actual_LD_sub);
    fprintf('     - Score = min(10, (Actual/Target)*10) = min(10, (%.2f/%.1f)*10) = %.1f -> %d/10\n', actual_LD_sub, target_LD_sub, score_sub, round(score_sub));
    
    fprintf('\n   Metric 2: Transonic Drag Rise (dCD0)\n');
    actual_dCD0 = max(Results.CD0(mach_vector > 0.8 & mach_vector < 1.2),[],'all') - AC.Aero.CD0_subsonic_base;
    score_trans = min(10, max(0, (1 - actual_dCD0 / target_dCD0) * 10));
    fprintf('     - Actual = max CD0 for M in [0.9, 1.1] - Base CD0 = %.4f - %.4f = %.4f\n', max(Results.CD0(mach_vector > 0.8 & mach_vector < 1.2),[],'all'), AC.Aero.CD0_subsonic_base, actual_dCD0);
    fprintf('     - Score = min(10, (1 - Actual/Target)*10) = min(10, (1 - %.4f/%.4f)*10) = %.1f -> %d/10\n', actual_dCD0, target_dCD0, score_trans, round(score_trans));

    fprintf('\n   Metric 3: Supersonic L/D\n');
    actual_LD_sup = 0; if any(mach_vector > 1.5), actual_LD_sup = mean(Results.LD_max(mach_vector > 1.5),'all','omitnan'); end
    score_sup = min(10, max(0, (actual_LD_sup / target_LD_sup) * 10));
    fprintf('     - Actual = mean L/D for M > 1.5 = %.2f\n', actual_LD_sup);
    fprintf('     - Score = min(10, (Actual/Target)*10) = min(10, (%.2f/%.1f)*10) = %.1f -> %d/10\n', actual_LD_sup, target_LD_sup, score_sup, round(score_sup));

    fprintf('\n   Metric 4: Energy Maneuverability (Ps)\n');
    fprintf('     - Ps calculated at M=0.8, 5000m, 90%% TOW (%.0f kg)\n', AC.Weight.TOW_guess*0.9);
    combat_atmo = isa_model(5000,false);
    [T_combat, ~] = get_engine_thrust(AC.Prop.engine_class, 0.8, combat_atmo, AC.Prop.num_engines, false);
    fprintf('     - Thrust available (T_combat) = %.0f N\n', T_combat);
    D_combat = calculate_drag_at_point(AC, 0.8, 5000, AC.Weight.TOW_guess * 0.9, true);
    V_combat = 0.8 * combat_atmo.a;
    fprintf('     - Velocity (V_combat) = %.2f m/s\n', V_combat);
    actual_Ps = (T_combat - D_combat) * V_combat / (AC.Weight.TOW_guess * 0.9 * g);
    score_Ps = min(10, max(0, (actual_Ps / target_Ps) * 10));
    fprintf('     - Actual Ps = (T-D)*V/W = (%.0f - %.0f)*%.2f / (%.0f*9.81) = %.2f m/s\n', T_combat, D_combat, V_combat, AC.Weight.TOW_guess*0.9, actual_Ps);
    fprintf('     - Score = min(10, (Actual/Target)*10) = min(10, (%.2f/%.1f)*10) = %.1f -> %d/10\n', actual_Ps, target_Ps, score_Ps, round(score_Ps));
    
    fprintf('-------------------------------------------------------------------------\n'); fprintf('| Metric                      | Target    | Actual    | Score     |\n'); fprintf('-------------------------------------------------------------------------\n');
    fprintf('| Subsonic Efficiency (L/D)   | >%-8.1f | %-9.2f | %-9s |\n', target_LD_sub, actual_LD_sub, sprintf('%d/10', round(score_sub))); fprintf('| Transonic Drag Rise (dCD0)  | <%-8.4f | %-9.4f | %-9s |\n', target_dCD0, actual_dCD0, sprintf('%d/10', round(score_trans))); fprintf('| Supersonic Efficiency (L/D) | >%-8.1f | %-9.2f | %-9s |\n', target_LD_sup, actual_LD_sup, sprintf('%d/10', round(score_sup))); fprintf('| Energy Maneuverability (Ps) | >%-8.1f | %-9.2f | %-9s |\n', target_Ps, actual_Ps, sprintf('%d/10', round(score_Ps))); fprintf('-------------------------------------------------------------------------\n\n');
    
    % --- Compliance Check Calculations ---
    fprintf('*** 3. DESIGN REQUIREMENTS COMPLIANCE CHECK ***\n');
    Stall_WS_req = CData.Stall.WS_max / g;
    fprintf('1. Stall Speed (<%d kts): Required W/S < %.1f. Design has W/S = %.1f. -> %s (%.1f < %.1f)\n', AC.Req.V_stall_max_kts, Stall_WS_req, W0_S, iif(W0_S < Stall_WS_req, 'PASS', 'FAIL'), W0_S, Stall_WS_req);
    TW_req_takeoff = interp1(CData.WS_vector, CData.Takeoff.TW, W0_S, 'linear', 'extrap');
    fprintf('2. Takeoff Dist (<%d m):  Requires T/W > %.3f. Design has T/W = %.3f.  -> %s (Margin: %+.3f)\n', AC.Req.Takeoff_dist_m, TW_req_takeoff, T0_W0, iif(T0_W0 > TW_req_takeoff, 'PASS', 'FAIL'), T0_W0 - TW_req_takeoff);
    TW_req_turn = interp1(CData.WS_vector, CData.Turn.TW, W0_S, 'linear', 'extrap');
    fprintf('3. Sustained Turn (>%.1f g): Requires T/W > %.3f. Design has T/W = %.3f.  -> %s (Margin: %+.3f)\n', AC.Req.Sustained_Turn_g, TW_req_turn, T0_W0, iif(T0_W0 > TW_req_turn, 'PASS', 'FAIL'), T0_W0 - TW_req_turn);
    if AC.Req.Supercruise_mach > 0
        TW_req_sc = interp1(CData.WS_vector, CData.Supercruise.TW, W0_S, 'linear', 'extrap');
        [~, T0_mil] = get_engine_thrust(AC.Prop.engine_class, 0, isa_model(0, false), AC.Prop.num_engines, false);
        T0_W0_mil = T0_mil / (AC.Weight.TOW_guess*g);
        fprintf('   - Sea Level Static Mil Thrust-to-Weight = %.0f N / (%.0f kg * 9.81) = %.3f\n', T0_mil, AC.Weight.TOW_guess, T0_W0_mil);
        fprintf('4. Supercruise (M%.1f):    Requires Mil T/W > %.3f. Design has Mil T/W = %.3f. -> %s (Margin: %+.3f)\n', AC.Req.Supercruise_mach, TW_req_sc, T0_W0_mil, iif(T0_W0_mil > TW_req_sc, 'PASS', 'FAIL'), T0_W0_mil - TW_req_sc);
    end
    fprintf('\n*** 4. OVERALL DESIGN VERDICT ***\n');
    pass_count = (W0_S < Stall_WS_req) + (T0_W0 > TW_req_takeoff) + (T0_W0 > TW_req_turn); num_reqs = 3;
    if AC.Req.Supercruise_mach > 0, pass_count = pass_count + (T0_W0_mil > TW_req_sc); num_reqs = 4; end
    if pass_count == num_reqs, fprintf('VIABLE: The current design point meets all specified performance requirements.\nStrengths appear to be in areas with high scores and large requirement margins.\n');
    else, fprintf('NEEDS REVISION: The design FAILED %d of %d key requirements.\nConsider adjusting T/W (engine thrust) or W/S (wing size) to meet constraints.\n', num_reqs-pass_count, num_reqs); end
    fprintf('\n*** 5. KEY ASSUMPTIONS ***\n');
    fprintf('[A1] Incompressible lift-slope is 2*pi (Thin Airfoil Theory).\n');
    fprintf('[A2] Wave drag is modeled with an empirical rise past an M_crit estimated from t/c and sweep.\n');
    fprintf('[A3] Oswald efficiency `e` is from Raymer`s generic empirical fit.\n');
    fprintf('[A4] Engine performance is from a generic, class-based model, not a specific engine deck.\n\n');
end


% =========================================================================
% FUNCTION 3: AERODYNAMIC CALCULATOR
% =========================================================================
function Aero = calculate_aerodynamics(AC, mach, Atmo, verbose)
    if nargin < 4, verbose = false; end
    if verbose, fprintf('\n>> CALC (calculate_aerodynamics): Starting aerodynamic calculations...\n'); end
    
    cl_alpha_0 = 2*pi;
    if mach < 1.0
        beta = sqrt(1-mach^2);
        cl_alpha = cl_alpha_0 / beta;
        if verbose, fprintf('   - Subsonic regime (M=%.2f):\n', mach);
                      fprintf('     beta = sqrt(1 - M^2) = sqrt(1 - %.2f^2) = %.4f\n', mach, beta);
                      fprintf('     2D Lift Slope (cl_alpha) = 2*pi / beta = 6.2832 / %.4f = %.4f /rad\n', beta, cl_alpha); end
    else
        beta = sqrt(mach^2-1); if beta==0, beta=1e-6; end
        cl_alpha = 4 / beta;
        if verbose, fprintf('   - Supersonic regime (M=%.2f):\n', mach);
                      fprintf('     beta = sqrt(M^2 - 1) = sqrt(%.2f^2 - 1) = %.4f\n', mach, beta);
                      fprintf('     2D Lift Slope (cl_alpha) = 4 / beta = 4 / %.4f = %.4f /rad\n', beta, cl_alpha); end
    end
    
    Aero.CL_alpha = cl_alpha / (1 + cl_alpha / (pi * AC.Wing.AR));
    if verbose, fprintf('   - 3D Lift Slope (CL_alpha) = cl_alpha / (1 + cl_alpha/(pi*AR)) = %.4f / (1 + %.4f/(pi*%.2f)) = %.4f /rad\n', cl_alpha, cl_alpha, AC.Wing.AR, Aero.CL_alpha); end
    
    CL_max_clean_subsonic = AC.Wing.cl_max_base * cosd(AC.Wing.Lambda_c4_deg);
    if verbose, fprintf('   - Base Clean CL_max (subsonic) = cl_max_base * cos(sweep) = %.2f * cos(%.1f) = %.4f\n', AC.Wing.cl_max_base, AC.Wing.Lambda_c4_deg, CL_max_clean_subsonic); end
    
    if mach<0.7, mach_factor=1.0; elseif mach<1.2, mach_factor=1-0.8*(mach-0.7); else, mach_factor=0.6; end
    if verbose, fprintf('   - Mach compressibility factor for CL_max = %.2f\n', mach_factor); end
    
    Aero.CL_max_clean = CL_max_clean_subsonic * mach_factor;
    if verbose, fprintf('   - Clean CL_max (at current Mach) = Base * factor = %.4f * %.2f = %.4f\n', CL_max_clean_subsonic, mach_factor, Aero.CL_max_clean); end
    
    if mach < 0.4
        Aero.CL_max = Aero.CL_max_clean + AC.HighLift.delta_CL_max_flap + AC.HighLift.delta_CL_max_slat;
        if verbose, fprintf('   - Total CL_max (with high-lift devices) = Clean + d_flap + d_slat = %.4f + %.2f + %.2f = %.4f\n', Aero.CL_max_clean, AC.HighLift.delta_CL_max_flap, AC.HighLift.delta_CL_max_slat, Aero.CL_max); end
    else
        Aero.CL_max = Aero.CL_max_clean;
        if verbose, fprintf('   - Total CL_max (clean config) = %.4f\n', Aero.CL_max); end
    end

    CD0 = AC.Aero.CD0_subsonic_base;
    M_crit_airfoil = 0.95 - AC.Wing.tc_avg*2;
    M_crit_wing = M_crit_airfoil / cosd(AC.Wing.Lambda_c4_deg)^0.5;
    if verbose, fprintf('   - Critical Mach (Mcrit) = (0.95-2*tc)/sqrt(cos(sweep)) = (0.95-2*%.3f)/sqrt(cos(%.1f)) = %.4f\n', AC.Wing.tc_avg, AC.Wing.Lambda_c4_deg, M_crit_wing); end
    
    if mach > M_crit_wing
        CD_wave_raw = 0.08*(mach-M_crit_wing)^2;
        if mach > 1.2
            max_transonic_wave_drag = 0.08*(1.2-M_crit_wing)^2;
            CD_wave = min(CD_wave_raw, max_transonic_wave_drag) / sqrt(mach);
            if verbose, fprintf('   - Wave Drag (CDw) = min(raw, max_trans)*M^-0.5 = min(%.4f, %.4f)/sqrt(%.2f) = %.4f\n', CD_wave_raw, max_transonic_wave_drag, mach, CD_wave); end
        else
            CD_wave = CD_wave_raw;
            if verbose, fprintf('   - Wave Drag (CDw) = 0.08*(M-Mcrit)^2 = 0.08*(%.2f-%.4f)^2 = %.4f\n', mach, M_crit_wing, CD_wave); end
        end
        CD0 = CD0 + CD_wave;
    elseif verbose
        fprintf('   - Wave Drag (CDw) = 0 (M < Mcrit)\n');
    end
    Aero.CD0 = CD0;
    if verbose, fprintf('   - Total Zero-Lift Drag (CD0) = Base + CDw = %.4f\n', Aero.CD0); end

    e_subsonic = 1.78*(1-0.045*AC.Wing.AR^0.68)-0.64;
    if verbose, fprintf('   - Oswald Efficiency (e) [subsonic formula] = 1.78*(1-0.045*%.2f^0.68)-0.64 = %.4f\n', AC.Wing.AR, e_subsonic); end
    
    if mach < 1.0
        Aero.K = 1/(pi*AC.Wing.AR*e_subsonic);
        if verbose, fprintf('   - Induced Drag Factor (K) [subsonic] = 1/(pi*AR*e) = 1/(pi*%.2f*%.4f) = %.4f\n', AC.Wing.AR, e_subsonic, Aero.K); end
    else
        Aero.K = Aero.CL_alpha / (pi*AC.Wing.AR); % Using leading-edge suction analogy
        if verbose, fprintf('   - Induced Drag Factor (K) [supersonic] = CL_alpha/(pi*AR) = %.4f/(pi*%.2f) = %.4f\n', Aero.CL_alpha, AC.Wing.AR, Aero.K); end
    end
end


% =========================================================================
% FUNCTION 4: ATMOSPHERE MODEL
% =========================================================================
function Atmo = isa_model(h, verbose)
    if nargin < 2, verbose = false; end
    if verbose, fprintf('\n>> CALC (isa_model): Calculating atmospheric properties at %.0f m...\n', h); end
    
    R=287.05; gamma=1.4; 
    if h < 11000 % Troposphere
        T0=288.15; p0=101325; lr=-0.0065;
        Atmo.T = T0 + lr*h;
        exp_term = -9.80665/(lr*R);
        Atmo.p = p0 * (Atmo.T/T0)^exp_term;
        if verbose, fprintf('   - Temp (T) = T0 + lapse*h = 288.15 + (-0.0065)*%.0f = %.2f K\n', h, Atmo.T);
                      fprintf('   - Press (p) = p0*(T/T0)^-g/(l*R) = 101325*(%.2f/288.15)^%.4f = %.1f Pa\n', Atmo.T, exp_term, Atmo.p); end
    else % Stratosphere
        T11=216.65; p11=22632; h11=11000;
        Atmo.T = T11;
        exp_term = -9.80665*(h-h11)/(R*Atmo.T);
        Atmo.p = p11 * exp(exp_term);
        if verbose, fprintf('   - Temp (T) = T_11km = %.2f K (constant)\n', Atmo.T);
                      fprintf('   - Press (p) = p_11km*exp(-g*(h-h11)/(R*T)) = 22632*exp(%.4f) = %.1f Pa\n', exp_term, Atmo.p); end
    end
    Atmo.rho = Atmo.p / (R*Atmo.T);
    Atmo.a = sqrt(gamma*R*Atmo.T);
    if verbose, fprintf('   - Density (rho) = p/(R*T) = %.1f/(287.05*%.2f) = %.4f kg/m^3\n', Atmo.p, Atmo.T, Atmo.rho);
                  fprintf('   - Speed of Sound (a) = sqrt(g*R*T) = sqrt(1.4*287.05*%.2f) = %.2f m/s\n', Atmo.T, Atmo.a); end
end


% =========================================================================
% FUNCTION 5: GENERIC ENGINE THRUST MODEL
% =========================================================================
function [T_max, T_mil] = get_engine_thrust(class, m, Atmo, num_eng, verbose)
    if nargin < 5, verbose = false; end
    if verbose, fprintf('\n>> CALC (get_engine_thrust): Calculating available thrust...\n'); end

    rho_sl = 1.225; % ISA Sea Level Density
    rho_ratio = Atmo.rho / rho_sl;
    if verbose, fprintf('   - Density Ratio (sigma) = rho_alt / rho_sl = %.4f / %.3f = %.4f\n', Atmo.rho, rho_sl, rho_ratio); end
    
    switch class
        case 'high_bypass_ab', T0_mil = 60000; T0_max = 98000;
        case 'low_bypass_ab',  T0_mil = 88000; T0_max = 125000;
        case 'high_bypass_dry',T0_mil = 180000;T0_max = 180000;
    end
    
    T_mil = num_eng * T0_mil * rho_ratio^0.8 * (1 + 0.3*m);
    if verbose, fprintf('   - Military Thrust = N * T0_mil * sigma^0.8 * (1+0.3*M) = %d*%.0f*%.4f^0.8*(1+0.3*%.2f) = %.0f N\n', num_eng, T0_mil, rho_ratio, m, T_mil); end
    
    T_max = num_eng * T0_max * rho_ratio^0.7 * (1 + 0.2*m^2);
    if verbose, fprintf('   - Max A/B Thrust  = N * T0_max * sigma^0.7 * (1+0.2*M^2) = %d*%.0f*%.4f^0.7*(1+0.2*%.2f^2) = %.0f N\n', num_eng, T0_max, rho_ratio, m, T_max); end
end


% =========================================================================
% FUNCTION 6 & 7: HELPER FUNCTIONS FOR REPORTING
% =========================================================================
function philosophy = get_philosophy(name)
    if contains(name, 'US-Style'), philosophy = 'Balanced performance, high T/W for energy maneuverability, and sustained turn capability.';
    elseif contains(name, 'Soviet-Style'), philosophy = 'Extreme maneuverability at low speeds and high AoA, raw thrust, and robust field performance.';
    elseif contains(name, 'Bomber'), philosophy = 'Long-range payload delivery at high subsonic or supersonic speeds, with emphasis on aerodynamic efficiency.';
    else, philosophy = 'Maximum payload capacity, short-field (STOL) capability, and high subsonic cruise efficiency.'; end
end

function [t_ld_s, t_dCD0, t_ld_ss, t_Ps] = get_scoring_targets(name)
    if contains(name, 'Fighter'), t_ld_s=9;   t_dCD0=0.010; t_ld_ss=4.5; t_Ps=150;
    elseif contains(name, 'Bomber'), t_ld_s=16;  t_dCD0=0.008; t_ld_ss=5.0; t_Ps=30;
    else, t_ld_s=18;  t_dCD0=0.020; t_ld_ss=1.0; t_Ps=20; end
end

function out = iif(condition, true_val, false_val)
    if condition, out = true_val; else, out = false_val; end
end

function D = calculate_drag_at_point(AC, M, h, W, verbose)
    if nargin < 5, verbose = false; end
    if verbose, fprintf('     - Calculating Drag at Point (M=%.1f, h=%.0f, W=%.0f)...\n', M,h,W); end
    g = 9.81; 
    Atmo = isa_model(h, false); 
    Aero = calculate_aerodynamics(AC, M, Atmo, false);
    
    q = 0.5 * Atmo.rho * (M * Atmo.a)^2;
    if verbose, fprintf('       - Dynamic Pressure (q) = 0.5 * %.4f * (%.2f * %.2f)^2 = %.1f Pa\n', Atmo.rho, M, Atmo.a, q); end

    CL = W / (q * AC.Wing.S_ref);
    if verbose, fprintf('       - Required Lift Coeff (CL) = W / (q * S_ref) = %.0f / (%.1f * %.1f) = %.4f\n', W, q, AC.Wing.S_ref, CL); end

    CD = Aero.CD0 + Aero.K * CL^2;
    if verbose, fprintf('       - Drag Coeff (CD) = CD0 + K*CL^2 = %.4f + %.4f * %.4f^2 = %.4f\n', Aero.CD0, Aero.K, CL, CD); end

    D = q * AC.Wing.S_ref * CD;
    if verbose, fprintf('       - Total Drag (D) = q * S_ref * CD = %.1f * %.1f * %.4f = %.0f N\n', q, AC.Wing.S_ref, CD, D); end
end
