function u_v = compute_multiagent_apf(i, p_v_all, p_flock_goal, obstacles, A, Delta, params)
    % i: Index of the current robot
    % p_v_all: 2xN matrix of all virtual points
    % A: Adjacency matrix
    % Delta: 2xNxN matrix of desired relative positions (delta_star)
    
    p_v = p_v_all(:, i);
    N = size(p_v_all, 2);
    
    % 1. Attractive Force (Formation Consensus + Flock Goal)
    F_form = [0; 0];
    for j = 1:N
        if A(i, j) == 1
            % Pull toward the correct relative position
            p_vj = p_v_all(:, j);
            desired_offset = Delta(:, i, j); 
            F_form = F_form - params.k_form * ((p_v - p_vj) - desired_offset);
        end
    end
    
    % Weak global pull so the whole formation moves across the map
    F_global = -params.k_att * (p_v - p_flock_goal);
    F_att = F_form + F_global;
    
    F_rep = [0; 0];
    F_vortex = [0; 0];
    
    % 2. Repulsive and Vortex Forces (Obstacles)
    for k = 1:size(obstacles, 1)
        c = obstacles(k, 1:2)'; 
        r = obstacles(k, 3);    
        
        dist_center = norm(p_v - c);
        d = dist_center - r; 
        
        if d > 0 && d < params.d0
            grad_d = (p_v - c) / dist_center;
            rep_mag = params.k_rep * (1/d - 1/params.d0) * (1/d^2);
            F_rep_k = rep_mag * grad_d;
            
            % 90-degree CCW rotation for the vortex
            R_90 = [0, -1; 1, 0];
            F_vortex_k = params.k_vortex * R_90 * F_rep_k;
            
            F_rep = F_rep + F_rep_k;
            F_vortex = F_vortex + F_vortex_k;
        end
    end
    
    % 3. Connectivity Barrier Force (The Bungee Cord)
    F_conn = [0; 0];
    for j = 1:N
        if A(i, j) == 1 
            p_vj = p_v_all(:, j);
            dist_ij = norm(p_v - p_vj);
            
            safety_margin = 0.8 * params.d_th; 
            if dist_ij > safety_margin && dist_ij < params.d_th
                barrier_mag = (2 * dist_ij) / (params.d_th^2 - dist_ij^2)^2;
                grad_conn = (p_v - p_vj) / dist_ij;
                F_conn = F_conn - params.k_conn * barrier_mag * grad_conn;
            end
        end
    end
    
    % Total desired velocity for the virtual point
    u_v = F_att + F_rep + F_vortex + F_conn;
end