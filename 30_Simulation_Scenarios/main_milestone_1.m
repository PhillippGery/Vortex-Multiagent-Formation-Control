% main_milestone_1.m
clear; clc; close all;

% Add paths so MATLAB can find your functions
addpath('../20_Core_Math');
addpath('../40_Utilities');

%% 1. Initialization
dt = 0.05;              % Time step
t_max = 30;             % Max simulation time
N_steps = t_max / dt;

% Robot Parameters
q = [-4; 0; 0];         % Initial state [x; y; theta]
p_goal = [4; 0];        % Goal directly across the map
l = 0.2;                % Look-ahead distance (The Carrot)
v_max = 1.0;            % Max linear speed
w_max = 2.0;            % Max angular speed
robot_radius = 0.3;     % For visualization

% APF Parameters
params.k_att = 1.0;
params.k_rep = 0.5;
params.k_vortex = 1.5;  % High vortex gain to force the sweep
params.d0 = 2.0;        % Obstacle influence radius

% Environment: Single obstacle exactly in the way (Local minimum trap!)
obstacles = [0, 0, 1.0]; % [x, y, radius]
bounds = [-5, 5, -3, 3];

%% 2. Setup Figure
figure('Name', 'Milestone 1: Vortex Evasion', 'Position', [100, 100, 800, 500]);
plot_environment(obstacles, bounds);
plot(p_goal(1), p_goal(2), 'gx', 'MarkerSize', 15, 'LineWidth', 3); % Draw Goal

%% 3. Simulation Loop
for k = 1:N_steps
    % Extract current state
    x = q(1); y = q(2); th = q(3);
    
    % Calculate Look-ahead point
    p_v = [x + l*cos(th); 
           y + l*sin(th)];
           
    % Compute APF at the look-ahead point
    u_v = compute_total_apf(p_v, p_goal, obstacles, params);
    
    % Unicycle Dynamics (Feedback Linearization & Saturation)
    dq = unicycle_dynamics(q, u_v, l, v_max, w_max);
    
    % Euler Integration
    q = q + dq * dt;
    
    % Update Animation
    cla; % Clear axis
    plot_environment(obstacles, bounds);
    plot(p_goal(1), p_goal(2), 'gx', 'MarkerSize', 15, 'LineWidth', 3);
    animate_robots(q, l, robot_radius);
    drawnow;
    
    % Stopping condition
    if norm(q(1:2) - p_goal) < 0.1
        disp('Goal Reached!');
        break;
    end
end