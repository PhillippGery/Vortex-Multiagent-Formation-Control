function obstacles = generate_clustered_obstacles(N, max_dist, start_pos, goal_pos)
    % Generates N random circular obstacles in a central danger zone.
    % Guarantees every gap between obstacle surfaces is either:
    %   (a) a solid merge (overlap > 0.15 m), OR
    %   (b) a passable corridor (gap >= MIN_PASSABLE_GAP)
    % This eliminates impassable "corner traps" where the gap is between
    % 0 and robot diameter.
    %
    % Parameters:
    %   N          - Number of obstacles to place
    %   max_dist   - (unused, kept for API compatibility)
    %   start_pos  - [x;y] of the swarm start — kept clear
    %   goal_pos   - [x;y] of the goal — kept clear

    % Minimum surface-to-surface gap a robot can squeeze through
    % = 2 * robot_radius + 0.1m safety margin = 2*0.2 + 0.1 = 0.5m
    MIN_PASSABLE_GAP = 0.5;

    % An overlap deeper than this is treated as a "solid wall merge"
    MAX_MERGE_OVERLAP = 0.15;

    obstacles = zeros(N, 3);
    i = 1;
    max_attempts = 10000; % Safety counter to avoid infinite loops
    attempts = 0;

    % Define the Danger Zone (central region of the map)
    x_min = -6; x_max = 6;
    y_min = -6; y_max = 6;

    while i <= N && attempts < max_attempts
        attempts = attempts + 1;

        % Sample a random position and radius
        new_x = x_min + (x_max - x_min) * rand();
        new_y = y_min + (y_max - y_min) * rand();
        new_r = 0.2 + 0.35 * rand(); % Radius between 0.20 m and 0.55 m

        % --- Check 1: Keep clear of spawn and goal ---
        dist_to_start = norm([new_x; new_y] - start_pos);
        dist_to_goal  = norm([new_x; new_y] - goal_pos);
        if dist_to_start < 3.0 || dist_to_goal < 3.0
            continue;
        end

        % --- Check 2: Passable-gap enforcement ---
        % For every previously placed obstacle j, the surface-to-surface gap
        % surface_gap = dist(centers) - r_new - r_j must satisfy:
        %   surface_gap < -MAX_MERGE_OVERLAP  (solid wall merge), OR
        %   surface_gap >= MIN_PASSABLE_GAP   (robot can pass through)
        % Anything in between creates an impassable corner and is REJECTED.
        valid = true;
        for j = 1:(i-1)
            dist_centers = norm([new_x; new_y] - [obstacles(j,1); obstacles(j,2)]);
            surface_gap  = dist_centers - new_r - obstacles(j,3);

            in_impassable_zone = (surface_gap > -MAX_MERGE_OVERLAP) && ...
                                 (surface_gap <  MIN_PASSABLE_GAP);
            if in_impassable_zone
                valid = false;
                break;
            end
        end

        if valid
            obstacles(i, :) = [new_x, new_y, new_r];
            i = i + 1;
        end
    end

    % If we couldn't place all N obstacles without violating the gap constraint,
    % trim the array to however many were successfully placed.
    if i <= N
        obstacles = obstacles(1:i-1, :);
        warning('generate_clustered_obstacles: only placed %d/%d obstacles (gap constraint active).', i-1, N);
    end
end
