% run_pso_tuner.m
clear; clc;

addpath('../20_Core_Math');
addpath('../40_Utilities');

disp('Initializing Particle Swarm Optimization...');
disp('Deploying 30 particles over 50 generations. Please wait...');

% Variables to tune: [k_form, k_att, k_rep, k_vortex, k_conn]
nvars = 5;

% Lower Bounds (We don't want negative gains, and we need non-zero values)
lb = [0.5,  0.1,  0.1,  0.1,  0.5];

% Upper Bounds (Prevent gains from exploding and breaking the ODE solver)
ub = [5.0,  1.5,  3.0,  3.0,  3.0];

% PSO Options: 30 parameter combinations, evolved over 50 iterations
options = optimoptions('particleswarm', ...
    'SwarmSize', 30, ...
    'MaxIterations', 50, ...
    'Display', 'iter');

% Run the optimizer
[opt_params, best_score] = particleswarm(@evaluate_formation_cost, nvars, lb, ub, options);

disp('========================================');
disp('OPTIMIZATION COMPLETE');
fprintf('Best Penalty Score: %.2f\n', best_score);
disp('--- OPTIMAL GAINS ---');
fprintf('params.k_form   = %.3f;\n', opt_params(1));
fprintf('params.k_att    = %.3f;\n', opt_params(2));
fprintf('params.k_rep    = %.3f;\n', opt_params(3));
fprintf('params.k_vortex = %.3f;\n', opt_params(4));
fprintf('params.k_conn   = %.3f;\n', opt_params(5));
disp('========================================');