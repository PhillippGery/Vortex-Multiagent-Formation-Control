function animate_robots(q, l, robot_radius)
    % q is a 3xN matrix of robot states: [x; y; theta]
    % l is the look-ahead distance
    
    % Dimensions based on robot_radius to keep it proportional
    chassis_L = robot_radius * 1.5;
    chassis_W = robot_radius * 1.2;
    wheel_L = robot_radius * 0.8;
    wheel_W = robot_radius * 0.3;
    
    % Local frame definitions (centered at 0,0, pointing in +x direction)
    % Chassis (Square/Rectangle)
    chassis = [ chassis_L/2,  chassis_L/2, -chassis_L/2, -chassis_L/2;
                chassis_W/2, -chassis_W/2, -chassis_W/2,  chassis_W/2];
                
    % Left Wheel
    l_wheel = [ wheel_L/2,  wheel_L/2, -wheel_L/2, -wheel_L/2;
                wheel_W/2, -wheel_W/2, -wheel_W/2,  wheel_W/2];
    l_wheel(2,:) = l_wheel(2,:) + chassis_W/2 + wheel_W/2; % Shift left
    
    % Right Wheel
    r_wheel = [ wheel_L/2,  wheel_L/2, -wheel_L/2, -wheel_L/2;
                wheel_W/2, -wheel_W/2, -wheel_W/2,  wheel_W/2];
    r_wheel(2,:) = r_wheel(2,:) - chassis_W/2 - wheel_W/2; % Shift right
    
    for i = 1:size(q, 2)
        x = q(1, i);
        y = q(2, i);
        th = q(3, i);
        
        % Rotation matrix for heading
        R = [cos(th), -sin(th); 
             sin(th),  cos(th)];
             
        % Rotate and translate to global frame
        chassis_global = R * chassis + [x; y];
        l_wheel_global = R * l_wheel + [x; y];
        r_wheel_global = R * r_wheel + [x; y];
        
        % Draw the robot components
        % Chassis (Silver/Grey)
        fill(chassis_global(1,:), chassis_global(2,:), [0.8 0.8 0.8], 'EdgeColor', 'k');
        
        % Wheels (Black)
        fill(l_wheel_global(1,:), l_wheel_global(2,:), 'k');
        fill(r_wheel_global(1,:), r_wheel_global(2,:), 'k');
        
        % Draw a line connecting the center to the look-ahead point
        x_v = x + l * cos(th);
        y_v = y + l * sin(th);
        plot([x, x_v], [y, y_v], 'k-', 'LineWidth', 1.5);
        
        % Draw the actual "Carrot" (Orange dot)
        plot(x_v, y_v, '.', 'Color', [1, 0.5, 0], 'MarkerSize', 20);
    end
end