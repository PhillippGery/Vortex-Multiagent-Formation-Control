% main_milestone_3.m
clear; clc; close all;

addpath('../20_Core_Math');
addpath('../40_Utilities');
addpath('../50_Optimzation');

%% 1. Initialization
dt = 0.05;              
t_max = 60;             
N_steps = t_max / dt;

plot_field = true;


% N=5 Robots: Initialized in a random cluster
q = [-6, -6, -5, -7, -5;   % x
      0,  1, -1,  2, -2;   % y
      0,  0,  0,  0,  0];  % theta
N_robots = size(q, 2);

p_flock_goal = [6; 0]; % The whole formation drives here

l = 0.2;                
v_max = 1.8;            
w_max = 2.5;            
robot_radius = 0.2;     

% APF Parameters (Normalized Math Baseline)
params.k_form = 0.8;    % Followers actively drive themselves to hold shape
params.k_att = 40.5;     % Constant tractor beam for the leader
params.k_rep = 2.0;     % Solid wall pushback
params.k_vortex = 1.4;  % Rotational slide to squeeze through gaps
params.k_conn = 4.0;    % Sturdy bungee cord
params.d0 = 1.5;        
params.d_th = 1.8;
max_allowable_pull = 30.0;

params.k_form   = 1.304;
params.k_att    = 30.399;
params.k_rep    = 0.957;
params.k_vortex = 3.000;
params.k_conn   = 0.500;

% APF Parameters (Adaptive Architecture)
params.k_form   = 1.000;
params.k_att    = 6.000;
params.k_rep    = 0.786;
params.k_vortex = 2.170;
params.k_conn   = 0.500;

params.d0 = 1.5;        
params.d_th = 1.8;



% Environment: Cluttered warehouse map
obstacles = [ 0,  1.8, 0.8;
              %0,  0.0, 0.8;
              0, -1.8, 0.8;
              0, -2.8, 0.8;
              0, -3.8, 0.8;
              0, -4.8, 0.8;];


bounds = [-8, 8, -5, 5];

%% 2. Define the V-Formation Delta Matrix
% Node 1 is the tip. Nodes 2,4 are top wing. Nodes 3,5 are bottom wing.
% Define absolute target positions for the V-shape relative to the center
V_shape = [ 0.0, -1.0, -1.0, -2.0, -2.0;  % x offsets
            0.0,  1.0, -1.0,  2.0, -2.0]*0.6; % y offsets

Delta = zeros(2, N_robots, N_robots);
for i = 1:N_robots
    for j = 1:N_robots
        % desired displacement vector from j to i
        Delta(:, i, j) = V_shape(:, i) - V_shape(:, j); 
    end
end

%% 3. Setup Figure
figure('Name', 'Milestone 3: V-Formation Navigation', 'Position', [100, 100, 800, 500]);
plot_environment(obstacles, bounds);

% Draw the vector field in the background
if plot_field
    plot_vector_field(p_flock_goal, obstacles, params, bounds);
end 

plot(p_flock_goal(1), p_flock_goal(2), 'gx', 'MarkerSize', 15, 'LineWidth', 3);
%% 4. Simulation Loop
for k = 1:N_steps
    % Calculate Look-ahead points
    p_v_all = zeros(2, N_robots);
    for i = 1:N_robots
        p_v_all(1, i) = q(1, i) + l*cos(q(3, i));
        p_v_all(2, i) = q(2, i) + l*sin(q(3, i));
    end
    
    % Compute Network
    [A, ~, lambda_2, edges] = compute_laplacian(p_v_all, params.d_th);
    
    % APF & Dynamics
    dq = zeros(3, N_robots);
    for i = 1:N_robots
        u_v = compute_multiagent_fc_apf(i, p_v_all, p_flock_goal, obstacles, A, Delta, params);
        dq(:, i) = unicycle_dynamics(q(:, i), u_v, l, v_max, w_max);
    end
    
    q = q + dq * dt;
    
    % Animation
    if mod(k, 2) == 0 
        cla; 
        plot_environment(obstacles, bounds);
        
        % ADD THE VECTOR FIELD RE-DRAW HERE
        if plot_field
            plot_vector_field(p_flock_goal, obstacles, params, bounds);
        end

        plot(p_flock_goal(1), p_flock_goal(2), 'gx', 'MarkerSize', 15, 'LineWidth', 3);
        
        for e = 1:size(edges, 1)
            r1 = edges(e, 1); r2 = edges(e, 2);
            plot([q(1,r1), q(1,r2)], [q(2,r1), q(2,r2)], 'g--', 'LineWidth', 1.0);
        end
        
        animate_robots(q, l, robot_radius);
        title(sprintf('V-Formation | \\lambda_2 = %.3f', lambda_2));
        drawnow;
    end
    
    if norm(mean(q(1:2, :), 2) - p_flock_goal) < 0.5
        disp('Formation Reached Goal Location!');
        break;
    end
end