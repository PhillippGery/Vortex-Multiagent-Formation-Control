function u_v = compute_multiagent_apf(i, p_v_all, p_goal, obstacles, A, params)
    % i: Index of the current robot
    % p_v_all: 2xN matrix of all virtual points
    % A: Adjacency matrix
    
    p_v = p_v_all(:, i);
    
    % 1. Attractive Force (To Goal/Formation)
    F_att = -params.k_att * (p_v - p_goal);
    
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
    N = size(p_v_all, 2);
    for j = 1:N
        if A(i, j) == 1 % If they are connected
            p_vj = p_v_all(:, j);
            dist_ij = norm(p_v - p_vj);
            
            % Only apply barrier if they are getting dangerously close to breaking the link
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