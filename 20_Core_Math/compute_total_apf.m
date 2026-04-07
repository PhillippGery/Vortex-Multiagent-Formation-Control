function u_v = compute_total_apf(p_v, p_goal, obstacles, params)
    % p_v: Look-ahead point [x; y]
    % p_goal: Target destination [x; y]
    % obstacles: [x_c, y_c, radius]
    % params: struct with k_att, k_rep, k_vortex, d0
    
    % 1. Attractive Force (To Goal)
    F_att = -params.k_att * (p_v - p_goal);
    
    F_rep = [0; 0];
    F_vortex = [0; 0];
    
    % 2. Repulsive and Vortex Forces (Obstacles)
    for i = 1:size(obstacles, 1)
        c = obstacles(i, 1:2)'; % Obstacle center
        r = obstacles(i, 3);    % Obstacle radius
        
        dist_center = norm(p_v - c);
        d = dist_center - r; % Distance to boundary
        
        if d > 0 && d < params.d0
            % Gradient of distance 
            grad_d = (p_v - c) / dist_center;
            
            % Magnitude of repulsion
            rep_mag = params.k_rep * (1/d - 1/params.d0) * (1/d^2);
            
            F_rep_i = rep_mag * grad_d;
            
            % Vortex rotation (90 degrees CCW)
            R_90 = [0, -1; 1, 0];
            F_vortex_i = params.k_vortex * R_90 * F_rep_i;
            
            F_rep = F_rep + F_rep_i;
            F_vortex = F_vortex + F_vortex_i;
        end
    end
    
    % Total desired velocity for the virtual point
    u_v = F_att + F_rep + F_vortex;
end