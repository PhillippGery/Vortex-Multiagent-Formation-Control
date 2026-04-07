% main_milestone_2.m
clear; clc; close all;

addpath('../20_Core_Math');
addpath('../40_Utilities');

%% 1. Initialization
dt = 0.05;              
t_max = 40;             
N_steps = t_max / dt;

% Robot Parameters (N=2)
q = [-4, -4;   % x
      0.5, -0.5; % y
      0,    0];  % theta
      
% Separate goals to pull them through the map
p_goals = [4,  4; 
           1, -1]; 

l = 0.2;                
v_max = 1.2;            
w_max = 2.5;            
robot_radius = 0.3;     

% APF & Graph Parameters
params.k_att = 1.0;
params.k_rep = 0.8;
params.k_vortex = 1.5;  
params.d0 = 2.0;        
params.d_th = 2.5;      % Max communication distance
params.k_conn = 0.5;    % Barrier strength

% Environment
obstacles = [0, 0, 1.2]; 
bounds = [-5, 5, -4, 4];

%% 2. Setup Figure
figure('Name', 'Milestone 2: Connectivity Barrier', 'Position', [100, 100, 800, 500]);
plot_environment(obstacles, bounds);
plot(p_goals(1,1), p_goals(2,1), 'gx', 'MarkerSize', 15, 'LineWidth', 3);
plot(p_goals(1,2), p_goals(2,2), 'gx', 'MarkerSize', 15, 'LineWidth', 3);

%% 3. Simulation Loop
N_robots = size(q, 2);

for k = 1:N_steps
    % 1. Calculate Look-ahead points for ALL robots
    p_v_all = zeros(2, N_robots);
    for i = 1:N_robots
        p_v_all(1, i) = q(1, i) + l*cos(q(3, i));
        p_v_all(2, i) = q(2, i) + l*sin(q(3, i));
    end
    
    % 2. Compute Laplacian and Connectivity Graph
    [A, L, lambda_2, edges] = compute_laplacian(p_v_all, params.d_th);
    
    % Abort if network disconnects
    if lambda_2 <= 1e-4 && N_robots > 1
        disp('FATAL ERROR: Network Disconnected!');
        break;
    end
    
    % 3. Calculate APF and update dynamics for each robot
    dq = zeros(3, N_robots);
    for i = 1:N_robots
        u_v = compute_multiagent_apf(i, p_v_all, p_goals(:, i), obstacles, A, params);
        dq(:, i) = unicycle_dynamics(q(:, i), u_v, l, v_max, w_max);
    end
    
    % 4. Euler Integration
    q = q + dq * dt;
    
    % 5. Update Animation
    if mod(k, 2) == 0 % Animate every other frame for speed
        cla; 
        plot_environment(obstacles, bounds);
        plot(p_goals(1,1), p_goals(2,1), 'gx', 'MarkerSize', 15, 'LineWidth', 3);
        plot(p_goals(1,2), p_goals(2,2), 'gx', 'MarkerSize', 15, 'LineWidth', 3);
        
        % Draw communication links
        for e = 1:size(edges, 1)
            r1 = edges(e, 1);
            r2 = edges(e, 2);
            plot([q(1,r1), q(1,r2)], [q(2,r1), q(2,r2)], 'g--', 'LineWidth', 1.5);
        end
        
        animate_robots(q, l, robot_radius);
        title(sprintf('Connectivity \\lambda_2 = %.3f', lambda_2));
        drawnow;
    end
    
    % Stopping condition
    dist_to_goals = vecnorm(q(1:2, :) - p_goals);
    if max(dist_to_goals) < 0.2
        disp('Mission Accomplished: Network Intact.');
        break;
    end
end