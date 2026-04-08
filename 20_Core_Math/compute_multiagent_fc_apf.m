function u_v = compute_multiagent_fc_apf(i, p_v_all, p_flock_goal, obstacles, A, Delta, params)
    p_v = p_v_all(:, i);
    N = size(p_v_all, 2);
    
    % ==========================================
    % MECHANIC 1: DYNAMIC FORMATION SQUEEZE
    % ==========================================
    % Find distance to the closest obstacle
    min_obs_dist = params.d0; 
    for k = 1:size(obstacles, 1)
        c = obstacles(k, 1:2)';
        r = obstacles(k, 3);
        d = norm(p_v - c) - r;
        if d > 0 && d < min_obs_dist
            min_obs_dist = d;
        end
    end
    
    % Scale k_form based on obstacle proximity
    % If far away, squeeze_factor = 1.0. If touching obstacle, squeeze_factor = 0.1
    squeeze_factor = min_obs_dist / params.d0; 
    squeeze_factor = max(0.1, squeeze_factor); % Never let it hit absolute zero
    dynamic_k_form = params.k_form * squeeze_factor;
    
    F_form = [0; 0];
    for j = 1:N
        if A(i, j) == 1
            p_vj = p_v_all(:, j);
            desired_offset = Delta(:, i, j); 
            F_form = F_form - dynamic_k_form * ((p_v - p_vj) - desired_offset);
        end
    end
    
    % ==========================================
    % MECHANIC 2: THE LEADER BRAKE
    % ==========================================
    if i == 1
        dynamic_k_att = params.k_att;
        
        % Check how far the most struggling follower is
        max_follower_dist = 0;
        for j = 1:N
            if A(i, j) == 1
                dist_ij = norm(p_v - p_v_all(:, j));
                if dist_ij > max_follower_dist
                    max_follower_dist = dist_ij;
                end
            end
        end
        
        d_safe = 0.85 * params.d_th; 
        if max_follower_dist > d_safe
            % If a follower enters the danger zone, scale the leader's engine down to 0
            brake_factor = (params.d_th - max_follower_dist) / (params.d_th - d_safe);
            brake_factor = max(0.0, min(1.0, brake_factor)); 
            dynamic_k_att = params.k_att * brake_factor;
        end
        
        dist_to_goal = norm(p_v - p_flock_goal);
        if dist_to_goal > 0.1
            F_global = -dynamic_k_att * (p_v - p_flock_goal) / dist_to_goal;
        else
            F_global = [0; 0];
        end
    else
        F_global = [0; 0];
    end
    
    F_att = F_form + F_global;
    
    % ==========================================
    % REPULSIVE & VORTEX FORCES
    % ==========================================
    F_rep = [0; 0];
    F_vortex = [0; 0];
    for k = 1:size(obstacles, 1)
        c = obstacles(k, 1:2)'; 
        r = obstacles(k, 3);    
        
        dist_center = norm(p_v - c);
        d = dist_center - r; 
        
        if d > 0 && d < params.d0
            grad_d = (p_v - c) / dist_center;
            rep_mag = params.k_rep * (1/d - 1/params.d0) * (1/d^2);
            F_rep_k = rep_mag * grad_d;
            
            R_90 = [0, 1; -1, 0];
            F_vortex_k = params.k_vortex * R_90 * F_rep_k;
            
            F_rep = F_rep + F_rep_k;
            F_vortex = F_vortex + F_vortex_k;
        end
    end
    
    % ==========================================
    % SATURATED CONNECTIVITY BARRIER
    % ==========================================
    F_conn = [0; 0];
    for j = 1:N
        if A(i, j) == 1 
            p_vj = p_v_all(:, j);
            dist_ij = norm(p_v - p_vj);
            
            d_safe = 0.85 * params.d_th; 
            if dist_ij > d_safe && dist_ij < params.d_th
                danger_ratio = (dist_ij - d_safe) / (params.d_th - d_safe);
                danger_ratio = min(danger_ratio, 0.99); % Prevent pure infinity
                
                barrier_mag = tan((pi / 2) * danger_ratio);
                barrier_mag = min(barrier_mag, 15.0); % The Shock Absorber
                
                grad_conn = (p_v - p_vj) / dist_ij;
                F_conn = F_conn - params.k_conn * barrier_mag * grad_conn;
            end
        end
    end
    
    u_v = F_att + F_rep + F_vortex + F_conn;
end