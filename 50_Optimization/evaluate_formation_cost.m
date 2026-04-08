function J_total = evaluate_formation_cost(x)
    % x maps to our 5 tunable APF parameters:
    % [k_form, k_att, k_rep, k_vortex, k_conn]
    params.k_form   = x(1);
    params.k_att    = x(2);
    params.k_rep    = x(3);
    params.k_vortex = x(4);
    params.k_conn   = x(5);
    
    % Fixed parameters
    l = 0.2; v_max = 1.2; w_max = 2.5;
    params.d0 = 1.5; params.d_th = 1.8; 
    N_robots = 5;
    p_flock_goal = [6; 0];
    dt = 0.1; 
    t_max = 35;
    N_steps = t_max / dt;
    
    % The physical map boundaries [x_min, x_max, y_min, y_max]
    bounds = [-8, 8, -5, 5]; 
    
    % V-shape Delta definition
    V_shape = [ 0.0, -1.0, -1.0, -2.0, -2.0; 
                0.0,  1.0, -1.0,  2.0, -2.0] * 0.8;
    Delta = zeros(2, N_robots, N_robots);
    for i = 1:N_robots
        for j = 1:N_robots
            Delta(:, i, j) = V_shape(:, i) - V_shape(:, j); 
        end
    end
    
    % Define multiple environments to force generalized intelligence
    env1 = [ 0, 1.5, 0.8; 0, -1.8, 0.8; 0, -2.8, 0.8; 0, -3.8, 0.8; 0, -4.8, 0.8]; % The Wall
    env2 = [ 1, 1.5, 1.0; 1, -1.5, 1.0;  3, 0.0, 0.8];
    
    % Env 3: The Dogleg (Forces them into the top wall, then bottom wall)
    env3 = [ -2, -5.0, 2.5;  
             -2, -2.5, 2.5; 
              2,  5.0, 2.0;  
              2,  2.5, 2.5];
    env4 = [ 0, 2.8, 0.8;
              0, 3.8, 0.8;
              0, 4.8, 0.8;
              0,  1.5, 0.8;
              %0,  0.0, 0.8;
              0, -1.8, 0.8;
              0, -2.8, 0.8;
              0, -3.8, 0.8;
              0, -4.8, 0.8;];
              
    environments = {env1, env2, env3, env4};
    
    J_total = 0;
    
    % Test parameters across all environments
    for env_idx = 1:length(environments)
        % BUG FIX: Un-hardcoded the obstacle selection
        obstacles = environments{env_idx};

        start_pos = [-10; 0];
        N_obstacles = 32;
        max_obs_dist = 2.5; % The choke-point constraint
        obstacles = generate_clustered_obstacles(N_obstacles, max_obs_dist, start_pos, p_flock_goal);
        bounds = [-12, 12, -8, 8];
        
        % Reset start position for each test
        q = [-6.0, -6.2, -6.2, -6.4, -6.4;
              0.0,  0.5, -0.5,  1.0, -1.0;
              0.0,  0.0,  0.0,  0.0,  0.0];
              
        J_env = 0;
        goal_reached = false;
        
        for k = 1:N_steps
            p_v_all = zeros(2, N_robots);
            for i = 1:N_robots
                p_v_all(1, i) = q(1, i) + l*cos(q(3, i));
                p_v_all(2, i) = q(2, i) + l*sin(q(3, i));
            end
            
            [A, ~, lambda_2, ~] = compute_laplacian(p_v_all, params.d_th);
            
            % FATAL ERROR 1: Network disconnected
            if lambda_2 <= 1e-4
                J_total = J_total + 500000; 
                break; 
            end
            
            % FATAL ERROR 2: Obstacle OR Wall Collision
            collision = false;
            for i = 1:N_robots
                % Check Obstacles
                for obs = 1:size(obstacles,1)
                    if norm(p_v_all(:,i) - obstacles(obs,1:2)') < obstacles(obs,3)
                        collision = true; break;
                    end
                end
                if collision, break; end
                
                % Check Walls (with a 0.2m safety margin for the robot chassis)
                if p_v_all(1,i) < bounds(1) + 0.2 || p_v_all(1,i) > bounds(2) - 0.2 || ...
                   p_v_all(2,i) < bounds(3) + 0.2 || p_v_all(2,i) > bounds(4) - 0.2
                    collision = true; break;
                end
            end
            
            if collision
                J_total = J_total + 50000; 
                break;
            end
            
            dq = zeros(3, N_robots);
            form_err_sum = 0;
            
            for i = 1:N_robots
                % Passed bounds into the controller so Mechanic 1 (Squeeze) triggers near walls
                u_v = compute_multiagent_fc_apf(i, p_v_all, p_flock_goal, obstacles, bounds, A, Delta, params);
                dq(:, i) = unicycle_dynamics(q(:, i), u_v, l, v_max, w_max);
                
                for j = 1:N_robots
                    if A(i,j) == 1
                        form_err_sum = form_err_sum + norm((p_v_all(:,i) - p_v_all(:,j)) - Delta(:,i,j));
                    end
                end
            end
            
            q = q + dq * dt;
            J_env = J_env + form_err_sum * dt;
            
            if norm(q(1:2, 1) - p_flock_goal) < 0.5
                goal_reached = true;
                break;
            end
        end
        
        % Massive penalty if it played it safe and stalled out
        if ~goal_reached
            J_env = J_env + 200000 + norm(q(1:2, 1) - p_flock_goal) * 1000;
        end
        % Explicitly punish total time taken (k is the step count where it finished)
        J_env = J_env + (k * dt * 100);

        J_total = J_total + J_env;
    end
end