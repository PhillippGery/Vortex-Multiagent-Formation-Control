% main_results_generator.m
% =========================================================================
% Automated Simulation Results Generator for Paper Section VI
%
% Runs two scenarios and saves publication-ready figures to:
%   ../60_Results/  (created automatically if missing)
%
% Figures produced (PDF vector graphics — zoom-safe, publication-ready):
%   fig1_trajectory_gate.pdf     - Top-down trajectory: Staggered Gate Field
%   fig2_trajectory_forest.pdf   - Top-down trajectory: Random Forest Gauntlet
%   fig3_formation_error.pdf     - ||e(t)|| time-series (both scenarios)
%   fig4_lambda2.pdf             - lambda_2(t) time-series (both scenarios)
%   results_summary.txt          - Numerical summary for paper table
%
% Requires MATLAB R2020a+ for exportgraphics (vector PDF output).
% =========================================================================
clear; clc; close all;

addpath('/20_Core_Math');
addpath('/40_Utilities');

% --- Output directories ---
out_dir    = '../60_Results';
readme_dir = fullfile(out_dir, 'readme_figs');
if ~exist(out_dir,    'dir'), mkdir(out_dir);    end
if ~exist(readme_dir, 'dir'), mkdir(readme_dir); end

% =========================================================================
%% SHARED PARAMETERS
% =========================================================================
rng(42); % Fixed seed for reproducibility

l       = 0.2;
v_max   = 1.8;
w_max   = 2.5;
robot_radius = 0.2;

params.d0      = 1.5;
params.d_th    = 1.8;
params.k_form  = 26.327;
params.k_att   = 32.240;
params.k_rep   = 2.100;
params.k_vortex= 0.214;
params.k_conn  = 17.679;

N_robots = 5;

% V-shape (scaled 0.6)
V_shape = [ 0.0, -1.0, -1.0, -2.0, -2.0;
            0.0,  1.0, -1.0,  2.0, -2.0] * 0.6;
Delta = zeros(2, N_robots, N_robots);
for ii = 1:N_robots
    for jj = 1:N_robots
        Delta(:,ii,jj) = V_shape(:,ii) - V_shape(:,jj);
    end
end

% =========================================================================
%% SCENARIO 1: STAGGERED GATE FIELD (Milestone 3)
% =========================================================================
fprintf('Running Scenario 1: Staggered Gate Field...\n');

dt1    = 0.05;
t_max1 = 60;
N_steps1 = t_max1 / dt1;

q1 = [-6, -6, -5, -7, -5;
       0,  1, -1,  2, -2;
       0,  0,  0,  0,  0];

p_goal1 = [6; 3];
bounds1 = [-8, 8, -5, 5];

% Fixed obstacle layout (staggered gates leaving ~1.4m gaps)
obstacles1 = [ 0,  2.8, 0.8;
               0,  3.8, 0.8;
               0,  4.8, 0.8;
               0,  1.4, 0.8;
               0, -1.8, 0.8;
               0, -2.8, 0.8;
               0, -3.8, 0.8;
               0, -4.8, 0.8];

% Storage
form_err1 = NaN(N_steps1, 1);
lambda2_1 = NaN(N_steps1, 1);
traj1     = NaN(N_steps1, 2, N_robots); % [step, xy, robot]
k1_end    = N_steps1;

for k = 1:N_steps1
    p_v_all = zeros(2, N_robots);
    for i = 1:N_robots
        p_v_all(1,i) = q1(1,i) + l*cos(q1(3,i));
        p_v_all(2,i) = q1(2,i) + l*sin(q1(3,i));
    end

    [A, ~, lam2, ~] = compute_laplacian(p_v_all, params.d_th);
    lambda2_1(k) = lam2;

    % Formation error
    ferr = 0;
    n_edges = 0;
    for i = 1:N_robots
        for j = 1:N_robots
            if A(i,j) == 1
                ferr = ferr + norm((p_v_all(:,i) - p_v_all(:,j)) - Delta(:,i,j))^2;
                n_edges = n_edges + 1;
            end
        end
    end
    form_err1(k) = sqrt(ferr / max(n_edges,1));

    traj1(k,:,:) = q1(1:2,:);

    dq = zeros(3, N_robots);
    for i = 1:N_robots
        u_v = compute_multiagent_fc_apf(i, p_v_all, p_goal1, obstacles1, bounds1, A, Delta, params);
        dq(:,i) = unicycle_dynamics(q1(:,i), u_v, l, v_max, w_max);
    end
    q1 = q1 + dq * dt1;

    if norm(mean(q1(1:2,:),2) - p_goal1) < 0.5
        k1_end = k;
        fprintf('  Scenario 1: Goal reached at t = %.2f s\n', k*dt1);
        break;
    end
end

t1 = (1:k1_end)' * dt1;
form_err1 = form_err1(1:k1_end);
lambda2_1 = lambda2_1(1:k1_end);

% =========================================================================
%% SCENARIO 2: RANDOM FOREST GAUNTLET (Milestone 4)
% =========================================================================
fprintf('Running Scenario 2: Random Forest Gauntlet...\n');

dt2    = 0.05;
t_max2 = 90;
N_steps2 = t_max2 / dt2;

q2 = [-10.0, -10.2, -10.2, -10.4, -10.4;
        0.0,   0.5,  -0.5,   1.0,  -1.0;
        0.0,   0.0,   0.0,   0.0,   0.0];

p_goal2  = [10; -5];
bounds2  = [-12, 12, -8, 8];
start2   = [-10; 0];
obstacles2 = generate_clustered_obstacles(32, 0.5, start2, p_goal2);

% Storage
form_err2 = NaN(N_steps2, 1);
lambda2_2 = NaN(N_steps2, 1);
traj2     = NaN(N_steps2, 2, N_robots);
k2_end    = N_steps2;

for k = 1:N_steps2
    p_v_all = zeros(2, N_robots);
    for i = 1:N_robots
        p_v_all(1,i) = q2(1,i) + l*cos(q2(3,i));
        p_v_all(2,i) = q2(2,i) + l*sin(q2(3,i));
    end

    [A, ~, lam2, ~] = compute_laplacian(p_v_all, params.d_th);
    lambda2_2(k) = lam2;

    ferr = 0; n_edges = 0;
    for i = 1:N_robots
        for j = 1:N_robots
            if A(i,j) == 1
                ferr = ferr + norm((p_v_all(:,i) - p_v_all(:,j)) - Delta(:,i,j))^2;
                n_edges = n_edges + 1;
            end
        end
    end
    form_err2(k) = sqrt(ferr / max(n_edges,1));
    traj2(k,:,:) = q2(1:2,:);

    dq = zeros(3, N_robots);
    for i = 1:N_robots
        u_v = compute_multiagent_fc_apf(i, p_v_all, p_goal2, obstacles2, bounds2, A, Delta, params);
        dq(:,i) = unicycle_dynamics(q2(:,i), u_v, l, v_max, w_max);
    end
    q2 = q2 + dq * dt2;

    if norm(mean(q2(1:2,:),2) - p_goal2) < 0.5
        k2_end = k;
        fprintf('  Scenario 2: Goal reached at t = %.2f s\n', k*dt2);
        break;
    end
end

if k2_end == N_steps2
    fprintf('  WARNING: Scenario 2 did not reach goal within %.0f s!\n', t_max2);
end

t2 = (1:k2_end)' * dt2;
form_err2 = form_err2(1:k2_end);
lambda2_2 = lambda2_2(1:k2_end);

% =========================================================================
%% FIGURE 1 & 2: TOP-DOWN TRAJECTORIES
% =========================================================================
colors = lines(N_robots);

% --- Figure 1: Gate Field ---
fig1 = figure('Name','Trajectory - Gate Field','Position',[50,50,700,450]);
hold on; axis equal; grid on;
set(gca,'FontSize',11);

% Draw obstacles
th_circ = linspace(0, 2*pi, 60);
for k = 1:size(obstacles1,1)
    fill(obstacles1(k,1) + obstacles1(k,3)*cos(th_circ), ...
         obstacles1(k,2) + obstacles1(k,3)*sin(th_circ), ...
         [0.5 0.5 0.5], 'EdgeColor','k','FaceAlpha',0.7);
end

% Draw bounds
rectangle('Position',[bounds1(1), bounds1(3), ...
           bounds1(2)-bounds1(1), bounds1(4)-bounds1(3)], ...
           'EdgeColor','k','LineWidth',2);

% Draw trajectories
for i = 1:N_robots
    x_traj = squeeze(traj1(1:k1_end,1,i));
    y_traj = squeeze(traj1(1:k1_end,2,i));
    plot(x_traj, y_traj, '-', 'Color', colors(i,:), 'LineWidth', 1.5);
    plot(x_traj(1), y_traj(1), 'o', 'Color', colors(i,:), ...
         'MarkerFaceColor', colors(i,:), 'MarkerSize', 7);
end

% Start, Goal markers
plot(q1(1,:), q1(2,:), 'ks', 'MarkerSize', 8, 'MarkerFaceColor','w','LineWidth',1.5);
plot(p_goal1(1), p_goal1(2), 'g*', 'MarkerSize', 15, 'LineWidth', 2.5);

xlabel('$x$ [m]','Interpreter','latex','FontSize',12);
ylabel('$y$ [m]','Interpreter','latex','FontSize',12);
title('Scenario I: Staggered Gate Field — Robot Trajectories','FontSize',11);
legend({'Obs','','','','','','R1','R2','R3','R4','R5','Start','Goal'}, ...
       'Location','northwest','FontSize',8);
xlim([bounds1(1) bounds1(2)]); ylim([bounds1(3) bounds1(4)]);

exportgraphics(fig1, fullfile(out_dir, 'fig1_trajectory_gate.pdf'), 'ContentType', 'vector');
exportgraphics(fig1, fullfile(out_dir, 'readme_figs', 'fig1_trajectory_gate-1.png'), 'Resolution', 180);
fprintf('Saved: fig1_trajectory_gate.pdf + .png\n');

% --- Figure 2: Random Forest ---
fig2 = figure('Name','Trajectory - Random Forest','Position',[50,50,800,500]);
hold on; axis equal; grid on;
set(gca,'FontSize',11);

for k = 1:size(obstacles2,1)
    fill(obstacles2(k,1) + obstacles2(k,3)*cos(th_circ), ...
         obstacles2(k,2) + obstacles2(k,3)*sin(th_circ), ...
         [0.5 0.5 0.5], 'EdgeColor','k','FaceAlpha',0.7);
end
rectangle('Position',[bounds2(1), bounds2(3), ...
           bounds2(2)-bounds2(1), bounds2(4)-bounds2(3)], ...
           'EdgeColor','k','LineWidth',2);

for i = 1:N_robots
    x_traj = squeeze(traj2(1:k2_end,1,i));
    y_traj = squeeze(traj2(1:k2_end,2,i));
    plot(x_traj, y_traj, '-', 'Color', colors(i,:), 'LineWidth', 1.5);
    plot(x_traj(1), y_traj(1), 'o', 'Color', colors(i,:), ...
         'MarkerFaceColor', colors(i,:), 'MarkerSize', 7);
end

plot(q2(1,:), q2(2,:), 'ks', 'MarkerSize', 8, 'MarkerFaceColor','w','LineWidth',1.5);
plot(p_goal2(1), p_goal2(2), 'g*', 'MarkerSize', 15, 'LineWidth', 2.5);

xlabel('$x$ [m]','Interpreter','latex','FontSize',12);
ylabel('$y$ [m]','Interpreter','latex','FontSize',12);
title('Scenario II: Random Forest Gauntlet — Robot Trajectories','FontSize',11);
xlim([bounds2(1) bounds2(2)]); ylim([bounds2(3) bounds2(4)]);

exportgraphics(fig2, fullfile(out_dir, 'fig2_trajectory_forest.pdf'), 'ContentType', 'vector');
exportgraphics(fig2, fullfile(out_dir, 'readme_figs', 'fig2_trajectory_forest-1.png'), 'Resolution', 180);
fprintf('Saved: fig2_trajectory_forest.pdf + .png\n');

% =========================================================================
%% FIGURE 3: FORMATION ERROR ||e(t)||
% =========================================================================
% Apply a short moving-average (window = 10 steps = 0.5 s) to remove the
% high-frequency ripple caused by binary edge-switching near d_th.
% The raw signal is unchanged; only the plotted line is smoothed.
w_smooth = 10;
fe1_smooth = movmean(form_err1, w_smooth);
fe2_smooth = movmean(form_err2, w_smooth);

fig3 = figure('Name','Formation Error','Position',[50,50,700,350]);
hold on; grid on;
set(gca,'FontSize',11);

% Light raw trace for reference, bold smoothed line on top
plot(t1, form_err1, 'b-', 'LineWidth', 0.6, 'Color', [0.5 0.7 1.0], ...
     'HandleVisibility','off');
plot(t2, form_err2, 'r-', 'LineWidth', 0.6, 'Color', [1.0 0.7 0.7], ...
     'HandleVisibility','off');
plot(t1, fe1_smooth, 'b-', 'LineWidth', 2.0, 'DisplayName', 'Scenario I (Gate Field)');
plot(t2, fe2_smooth, 'r-', 'LineWidth', 2.0, 'DisplayName', 'Scenario II (Random Forest)');

xlabel('Time $t$ [s]','Interpreter','latex','FontSize',12);
ylabel('$\|e(t)\|$ [m]','Interpreter','latex','FontSize',12);
title('Formation Error vs. Time','FontSize',11);
legend('Location','northeast','FontSize',10);
yline(0.1, 'k--', 'Converged threshold (0.1 m)', ...
      'LabelVerticalAlignment','bottom','FontSize',9);

exportgraphics(fig3, fullfile(out_dir, 'fig3_formation_error.pdf'), 'ContentType', 'vector');
exportgraphics(fig3, fullfile(out_dir, 'readme_figs', 'fig3_formation_error-1.png'), 'Resolution', 180);
fprintf('Saved: fig3_formation_error.pdf + .png\n');

% =========================================================================
%% FIGURE 4: ALGEBRAIC CONNECTIVITY lambda_2(t)
% =========================================================================
fig4 = figure('Name','Algebraic Connectivity','Position',[50,50,700,350]);
hold on; grid on;
set(gca,'FontSize',11);

plot(t1, lambda2_1, 'b-', 'LineWidth', 1.8, 'DisplayName', 'Scenario I (Gate Field)');
plot(t2, lambda2_2, 'r-', 'LineWidth', 1.8, 'DisplayName', 'Scenario II (Random Forest)');
yline(0, 'k--', '\lambda_2 = 0 (disconnected)', ...
      'LabelVerticalAlignment','bottom','FontSize',9,'Interpreter','tex');

xlabel('Time $t$ [s]','Interpreter','latex','FontSize',12);
ylabel('$\lambda_2(L(t))$','Interpreter','latex','FontSize',12);
title('Algebraic Connectivity vs. Time','FontSize',11);
legend('Location','southeast','FontSize',10);
ylim([-0.1, max([max(lambda2_1); max(lambda2_2)]) * 1.1]);

exportgraphics(fig4, fullfile(out_dir, 'fig4_lambda2.pdf'), 'ContentType', 'vector');
exportgraphics(fig4, fullfile(out_dir, 'readme_figs', 'fig4_lambda2-1.png'), 'Resolution', 180);
fprintf('Saved: fig4_lambda2.pdf + .png\n');

% =========================================================================
%% RESULTS SUMMARY (substitute into paper §VI)
% =========================================================================
fid = fopen(fullfile(out_dir, 'results_summary.txt'), 'w');
fprintf(fid, '=== SIMULATION RESULTS SUMMARY ===\n\n');

fprintf(fid, 'SCENARIO I - Staggered Gate Field\n');
fprintf(fid, '  Time to goal:          %.2f s\n', k1_end * dt1);
fprintf(fid, '  Peak formation error:  %.4f m\n', max(form_err1));
fprintf(fid, '  Final formation error: %.4f m\n', form_err1(end));
fprintf(fid, '  Min lambda_2:          %.6f\n',   min(lambda2_1));
fprintf(fid, '  lambda_2 ever <= 0:    %s\n',     mat2str(any(lambda2_1 <= 0)));

fprintf(fid, '\nSCENARIO II - Random Forest Gauntlet\n');
fprintf(fid, '  Time to goal:          %.2f s\n', k2_end * dt2);
fprintf(fid, '  Peak formation error:  %.4f m\n', max(form_err2));
fprintf(fid, '  Final formation error: %.4f m\n', form_err2(end));
fprintf(fid, '  Min lambda_2:          %.6f\n',   min(lambda2_2));
fprintf(fid, '  lambda_2 ever <= 0:    %s\n',     mat2str(any(lambda2_2 <= 0)));

fclose(fid);
fprintf('\nAll results saved to: %s\n', out_dir);
fprintf('Open results_summary.txt and substitute values into paper Section VI.\n');
