%--------------------------------------------------------------------------
% V-n Diagram Generator for Aircraft Structural Operating Envelope
%--------------------------------------------------------------------------
% This program calculates and plots the maneuver and gust V-n diagrams
% for an aircraft based on user-provided parameters. The calculations are
% derived from the standard flight mechanics equations provided.
%
% The program will:
%   1. Prompt the user for all necessary aircraft data.
%   2. Calculate key speeds and load factors.
%   3. Calculate the gust load envelope.
%   4. Generate a detailed and annotated V-n diagram.
%--------------------------------------------------------------------------

%% Clean Slate
clear all;
clc;
close all;

%% 1. User Inputs
% -------------------------------------------------------------------------
fprintf('--- Aircraft V-n Diagram Generator ---\n');
fprintf('Please provide the following aircraft parameters.\n');
fprintf('Press Enter to use the default values in [brackets].\n\n');

% --- Basic Aircraft Parameters ---
W_TO_max = input('Enter Maximum Gross Weight, W_TO_max (N) [e.g., 65000]: ');
if isempty(W_TO_max), W_TO_max = 65000; end

S_wing = input('Enter Wing Area, S_wing (m^2) [e.g., 35]: ');
if isempty(S_wing), S_wing = 35; end

b_wing = input('Enter Wing Span, b_wing (m) [e.g., 18]: ');
if isempty(b_wing), b_wing = 18; end

% --- Aerodynamic & Structural Limits ---
CL_max_pos = input('Enter Maximum Positive Lift Coefficient, C_Lmax,pos [e.g., 1.6]: ');
if isempty(CL_max_pos), CL_max_pos = 1.6; end

CL_max_neg = input('Enter Maximum Negative Lift Coefficient, C_Lmax,neg [optional, e.g., -1.1]: ');
if isempty(CL_max_neg)
    CL_max_neg = -0.7 * CL_max_pos; % Common assumption if not specified
    fprintf(' -> C_Lmax_neg not specified, assuming %.2f\n', CL_max_neg);
end

Nz_max_pos = input('Enter Maximum Positive Limit Load Factor, N_z,max,pos [e.g., 3.8]: ');
if isempty(Nz_max_pos), Nz_max_pos = 3.8; end

Nz_max_neg = input('Enter Maximum Negative Limit Load Factor, N_z,max,neg [optional, e.g., -1.5]: ');
if isempty(Nz_max_neg)
    Nz_max_neg = -0.4 * Nz_max_pos; % Common assumption if not specified
    fprintf(' -> N_z,max,neg not specified, assuming %.2f\n', Nz_max_neg);
end

% --- Flight & Gust Conditions ---
Vc = input('Enter Cruise Speed, Vc (m/s) [e.g., 220]: ');
if isempty(Vc), Vc = 220; end

Vd = input('Enter Dive Speed, Vd (m/s) [e.g., 275]: ');
if isempty(Vd)
    Vd = 1.25 * Vc; % Common certification requirement
    fprintf(' -> Vd not specified, assuming 1.25 * Vc = %.1f m/s\n', Vd);
end

AR_wing = input('Enter Wing Aspect Ratio, AR_wing [e.g., 9.0]: ');
if isempty(AR_wing), AR_wing = (b_wing^2)/S_wing; fprintf(' -> AR_wing calculated as %.2f\n', AR_wing); end

delta_m_deg = input('Enter Sweep of the max thickness line, delta_m (degrees) [e.g., 25]: ');
if isempty(delta_m_deg), delta_m_deg = 25; end

Vg_at_Vc = input('Enter Vertical Gust Speed at Vc (m/s) [e.g., 15.24 for 50 ft/s]: ');
if isempty(Vg_at_Vc), Vg_at_Vc = 15.24; end

Vg_at_Vd = input('Enter Vertical Gust Speed at Vd (m/s) [e.g., 7.62 for 25 ft/s]: ');
if isempty(Vg_at_Vd), Vg_at_Vd = 7.62; end

% --- Constants (as per documentation) ---
g = 9.81; % m/s^2
rho_SL = 1.225; % kg/m^3
a_sound_SL = 340.3; % Speed of sound at sea level, m/s

fprintf('\n--- Inputs confirmed. Starting calculations. ---\n\n');

%% 2. Calculations
% -------------------------------------------------------------------------

% --- A) Maneuver Envelope Speeds ---

% Eq (5.1) Stall Speed (Vs) in 1g level flight
Vs = sqrt((2 * W_TO_max * g) / (rho_SL * S_wing * CL_max_pos));

% Eq (5.2) Corner Speed (V_star or Va)
V_star = Vs * sqrt(Nz_max_pos);

% Eq (5.3) Speed at Maximum Negative Load Factor
V_at_neg_n = sqrt((2 * W_TO_max * g * abs(Nz_max_neg)) / (rho_SL * S_wing * abs(CL_max_neg)));


% --- B) Gust Envelope Calculations ---

% To calculate the gust load factor, we need to determine the gust
% alleviation factor (Kg) at the specified speeds (Vc and Vd). This
% requires calculating the lift curve slope (a), which is dependent on
% Mach number.

% Function to perform the gust calculation for a given speed
% This avoids repetitive code for Vc and Vd
function [delta_ng, a, mu_g, K_g] = calculate_gust_load(V, Vg, AR, delta_m_deg, S, b, W, rho, a_sound)
    g = 9.81;
    % Eq (5.6) Mean Wing Chord (c_bar)
    c_bar = S / b;
    
    % Eq (5.5) Beta (Prandtl-Glauert factor)
    M = V / a_sound;
    beta = sqrt(1 - M^2);
    
    % Eq (5.4) Lift Curve Slope (a) in per radian
    delta_m_rad = deg2rad(delta_m_deg);
    a = (2 * pi * AR) / (2 + sqrt(4 + AR^2 * beta^2 * (1 + (tan(delta_m_rad)^2) / beta^2)));
    
    % Eq (5.7) Mass Ratio Factor (mu_g)
    mu_g = (2 * W * g) / (a * rho * S * c_bar * g); % Note: g cancels out
    
    % Eq (5.8) Gust Alleviation Factor (K_g)
    K_g = (0.88 * mu_g) / (5.3 + mu_g);
    
    % Eq (5.9) Additional Gust Load Factor (delta_ng)
    delta_ng = (a * K_g * rho * V * Vg * S) / (2 * W * g);
end

% Calculate gust loads at Vc and Vd
[delta_ng_c, a_c, mu_g_c, K_g_c] = calculate_gust_load(Vc, Vg_at_Vc, AR_wing, delta_m_deg, S_wing, b_wing, W_TO_max, rho_SL, a_sound_SL);
[delta_ng_d, a_d, mu_g_d, K_g_d] = calculate_gust_load(Vd, Vg_at_Vd, AR_wing, delta_m_deg, S_wing, b_wing, W_TO_max, rho_SL, a_sound_SL);

% Eq (5.10) Total Gust Load Factors
N_gust_pos_c = 1 + delta_ng_c;
N_gust_neg_c = 1 - delta_ng_c;
N_gust_pos_d = 1 + delta_ng_d;
N_gust_neg_d = 1 - delta_ng_d;


%% 3. Display Results in Command Window
% -------------------------------------------------------------------------
fprintf('--- Calculation Summary ---\n');
fprintf('Maneuver Envelope Key Points:\n');
fprintf('  Stall Speed (Vs)                 : %.2f m/s (%.1f knots)\n', Vs, Vs*1.94384);
fprintf('  Corner Speed (Va)                : %.2f m/s (%.1f knots)\n', V_star, V_star*1.94384);
fprintf('  Speed at Max Negative Load (V_n-): %.2f m/s (%.1f knots)\n', V_at_neg_n, V_at_neg_n*1.94384);
fprintf('  Cruise Speed (Vc)                : %.2f m/s (%.1f knots)\n', Vc, Vc*1.94384);
fprintf('  Dive Speed (Vd)                  : %.2f m/s (%.1f knots)\n', Vd, Vd*1.94384);
fprintf('\n');
fprintf('Gust Envelope Calculations:\n');
fprintf('  At Cruise Speed (Vc = %.1f m/s):\n', Vc);
fprintf('    Lift Curve Slope (a)           : %.3f /rad\n', a_c);
fprintf('    Mass Ratio (mu_g)              : %.3f\n', mu_g_c);
fprintf('    Gust Alleviation Factor (K_g)  : %.3f\n', K_g_c);
fprintf('    Additional Gust Load (delta_ng): +/- %.3f\n', delta_ng_c);
fprintf('    Total Gust Load Factor (N_z)   : %.3f to %.3f\n', N_gust_neg_c, N_gust_pos_c);
fprintf('\n');
fprintf('  At Dive Speed (Vd = %.1f m/s):\n', Vd);
fprintf('    Lift Curve Slope (a)           : %.3f /rad\n', a_d);
fprintf('    Mass Ratio (mu_g)              : %.3f\n', mu_g_d);
fprintf('    Gust Alleviation Factor (K_g)  : %.3f\n', K_g_d);
fprintf('    Additional Gust Load (delta_ng): +/- %.3f\n', delta_ng_d);
fprintf('    Total Gust Load Factor (N_z)   : %.3f to %.3f\n', N_gust_neg_d, N_gust_pos_d);
fprintf('---------------------------------\n\n');


%% 4. Generate the V-n Diagram
% -------------------------------------------------------------------------
fprintf('Generating plot...\n');

figure('Name', 'V-n Diagram', 'NumberTitle', 'off', 'WindowState', 'maximized');
hold on;
grid on;
box on;

% --- A) Define and Plot Maneuver Envelope ---

% Stall Lines
V_pos_stall_range = linspace(0, V_star, 100);
n_pos_stall = (0.5 * rho_SL * V_pos_stall_range.^2 * S_wing * CL_max_pos) / (W_TO_max * g);

V_neg_stall_range = linspace(0, V_at_neg_n, 100);
n_neg_stall = (0.5 * rho_SL * V_neg_stall_range.^2 * S_wing * CL_max_neg) / (W_TO_max * g);

% Combine all points to create the envelope boundary
x_maneuver = [V_pos_stall_range, Vd, Vd, fliplr(V_neg_stall_range)];
y_maneuver = [n_pos_stall, Nz_max_pos, Nz_max_neg, fliplr(n_neg_stall)];

% Plot the filled maneuver envelope
fill(x_maneuver, y_maneuver, [0.7 0.85 1.0], 'EdgeColor', 'none', 'FaceAlpha', 0.6, 'DisplayName', 'Maneuver Envelope');

% Plot the boundary line for clarity
plot(x_maneuver, y_maneuver, 'b-', 'LineWidth', 2, 'DisplayName', 'Maneuver Limit');


% --- B) Define and Plot Gust Envelope ---

% The gust envelope is defined by lines connecting (0,1) to points at Vc and Vd
x_gust = [0, Vc, Vd];
y_gust_pos = [1, N_gust_pos_c, N_gust_pos_d];
y_gust_neg = [1, N_gust_neg_c, N_gust_neg_d];

% Plot the gust lines
plot(x_gust, y_gust_pos, 'r--', 'LineWidth', 2, 'DisplayName', 'Positive Gust Line');
plot(x_gust, y_gust_neg, 'r--', 'LineWidth', 2, 'HandleVisibility', 'off'); % Don't add to legend again

% Plot a filled region for the gust envelope
fill([x_gust, fliplr(x_gust)], [y_gust_pos, fliplr(y_gust_neg)], 'r', 'EdgeColor', 'none', 'FaceAlpha', 0.15, 'DisplayName', 'Gust Envelope');


% --- C) Annotations and Formatting ---

% Plot reference line for n=1 (level flight)
plot([0 Vd*1.1], [1 1], 'k:', 'LineWidth', 1, 'DisplayName', '1g Level Flight');

% Plot and label key speeds with vertical dashed lines
speeds = [Vs, V_star, Vc, Vd];
labels = {'V_s', 'V_a', 'V_c', 'V_d'};
colors = {'g', 'm', 'c', 'k'};
y_limits = ylim; % Get current y-axis limits for line plotting

for i = 1:length(speeds)
    plot([speeds(i), speeds(i)], [y_limits(1), y_limits(2)], [colors{i} '--']);
    text(speeds(i), y_limits(1) + 0.05 * (y_limits(2) - y_limits(1)), ...
        sprintf('  %s = %.1f m/s', labels{i}, speeds(i)), ...
        'Color', colors{i}, 'FontWeight', 'bold', 'FontSize', 10);
end

% Label key load factors
text(Vd*1.02, Nz_max_pos, sprintf('N_{z,max,pos} = %.2f', Nz_max_pos), 'HorizontalAlignment', 'left', 'VerticalAlignment', 'middle', 'Color', 'b', 'FontWeight', 'bold');
text(Vd*1.02, Nz_max_neg, sprintf('N_{z,max,neg} = %.2f', Nz_max_neg), 'HorizontalAlignment', 'left', 'VerticalAlignment', 'middle', 'Color', 'b', 'FontWeight', 'bold');

% --- D) Final Plot Polish ---
title('Aircraft Flight Envelope (V-n Diagram)', 'FontSize', 16, 'FontWeight', 'bold');
xlabel('Airspeed, V (m/s)', 'FontSize', 12);
ylabel('Load Factor, n_z', 'FontSize', 12);
legend('Location', 'northwest', 'FontSize', 10);
axis tight;
xlim([0 Vd * 1.15]); % Add some space to the right for labels
ylim_current = ylim;
ylim([ylim_current(1)-0.5, ylim_current(2)+0.5]); % Add some vertical space

hold off;

fprintf('Plot generated successfully.\n');
