function obstacles = generate_clustered_obstacles(N, max_dist, start_pos, goal_pos)
    % We are ignoring max_dist and instead scattering them across a "Danger Zone"
    obstacles = zeros(N, 3);
    i = 1;
    
    % Define the Danger Zone (The middle 12x12 meters of the map)
    x_min = -6; x_max = 6;
    y_min = -6; y_max = 6;
    
    while i <= N
        % Generate a random position within the Danger Zone
        new_x = x_min + (x_max - x_min) * rand();
        new_y = y_min + (y_max - y_min) * rand();
        new_r = 0.2 + 0.35 * rand(); % Radius between 0.5 and 1.1
        
        % 1. Safety Check: Keep them away from the spawn and the goal
        dist_to_start = norm([new_x; new_y] - start_pos);
        dist_to_goal = norm([new_x; new_y] - goal_pos);
        
        if dist_to_start < 3.0 || dist_to_goal < 3.0
            continue; % Too close to start/goal. Throw it out and try again.
        end
        
        % 2. Overlap Check: Ensure they don't spawn entirely inside each other
        % We allow them to touch or slightly overlap to create walls, but not stack completely.
        too_close = false;
        for j = 1:(i-1)
            dist_to_other = norm([new_x; new_y] - [obstacles(j, 1); obstacles(j, 2)]);
            % If the distance is less than their combined radii, they are overlapping
            if dist_to_other < (new_r + obstacles(j, 3) - 0.2) 
                too_close = true;
                break;
            end
        end
        
        % If it passed both safety checks, lock it in
        if ~too_close
            obstacles(i, :) = [new_x, new_y, new_r];
            i = i + 1; % Move on to the next obstacle
        end
    end
end