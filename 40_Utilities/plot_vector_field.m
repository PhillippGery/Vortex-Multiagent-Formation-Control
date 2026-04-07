function plot_vector_field(p_goal, obstacles, params, bounds)
    % Create a grid of points across the workspace
    % We use a 0.5 meter resolution so the screen isn't too cluttered
    x_range = bounds(1):0.5:bounds(2);
    y_range = bounds(3):0.5:bounds(4);
    [X, Y] = meshgrid(x_range, y_range);
    
    U = zeros(size(X));
    V = zeros(size(Y));
    
    for i = 1:size(X, 1)
        for j = 1:size(X, 2)
            p_v = [X(i, j); Y(i, j)];
            
            % 1. Attractive Force (Global Pull)
            F_att = -params.k_att * (p_v - p_goal);
            
            F_rep = [0; 0];
            F_vortex = [0; 0];
            
            % 2. Repulsive and Vortex Forces
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
            
            % Superposition of field vectors
            u_v = F_att + F_rep + F_vortex;
            
            % Normalize vectors to uniform length for a clean quiver plot
            mag = norm(u_v);
            if mag > 0.01
                u_v = u_v / mag; 
            end
            
            U(i, j) = u_v(1);
            V(i, j) = u_v(2);
        end
    end
    
    % Plot the vector field (0.4 scales the arrow lengths down slightly)
    quiver(X, Y, U, V, 0.4, 'Color', [0.7 0.7 0.7], 'LineWidth', 1);
end