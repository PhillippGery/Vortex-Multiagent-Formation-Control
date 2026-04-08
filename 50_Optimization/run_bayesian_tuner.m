% run_bayesian_tuner.m
clear; clc;

addpath('../20_Core_Math');
addpath('../40_Utilities');

disp('Initializing Bayesian Optimization...');
disp('Building surrogate model of the APF parameter space...');

% 1. Define the search space (Variables)
% Matching your PSO bounds: [k_form, k_att, k_rep, k_vortex, k_conn]
vars = [
    optimizableVariable('k_form',   [16.0, 40.0],  'Type', 'real')
    optimizableVariable('k_att',    [6.0, 50.0], 'Type', 'real')
    optimizableVariable('k_rep',    [0.1, 18.0],  'Type', 'real')
    optimizableVariable('k_vortex', [0.1, 18.0],  'Type', 'real')
    optimizableVariable('k_conn',   [0.5, 18.0],  'Type', 'real')
];

% 2. Define the objective function wrapper
obj_func = @(x) evaluate_formation_cost([x.k_form, x.k_att, x.k_rep, x.k_vortex, x.k_conn]);

% 3. Run Bayesian Optimization
% Removed 'DisplayRollback'. 
% Using 'Verbose', 1 for command window updates.
results = bayesopt(obj_func, vars, ...
    'MaxObjectiveEvaluations', 120, ...
    'Verbose', 1, ...
    'PlotFcn', {@plotObjectiveModel, @plotMinObjective}, ...
    'AcquisitionFunctionName', 'expected-improvement-plus');

% 4. Extract and Display Results
best_params = results.XAtMinObjective;
best_score = results.MinObjective;

disp('========================================');
disp('BAYESIAN OPTIMIZATION COMPLETE');
fprintf('Best Penalty Score: %.2f\n', best_score);
disp('--- OPTIMAL GAINS ---');
fprintf('params.k_form   = %.3f;\n', best_params.k_form);
fprintf('params.k_att    = %.3f;\n', best_params.k_att);
fprintf('params.k_rep    = %.3f;\n', best_params.k_rep);
fprintf('params.k_vortex = %.3f;\n', best_params.k_vortex);
fprintf('params.k_conn   = %.3f;\n', best_params.k_conn);
disp('========================================');