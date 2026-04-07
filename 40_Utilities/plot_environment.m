function plot_environment(obstacles, bounds)
    % bounds = [x_min, x_max, y_min, y_max]
    % obstacles = [x_c, y_c, radius; ...]
    
    axis equal;
    axis(bounds);
    hold on;
    grid on;
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    title('Vortex Multiagent Navigation');
    
    % Draw obstacles
    theta = linspace(0, 2*pi, 100);
    for i = 1:size(obstacles, 1)
        xc = obstacles(i, 1);
        yc = obstacles(i, 2);
        r = obstacles(i, 3);
        
        x_circle = r * cos(theta) + xc;
        y_circle = r * sin(theta) + yc;
        
        fill(x_circle, y_circle, [0.7 0.7 0.7], 'EdgeColor', 'k', 'FaceAlpha', 0.5);
    end
end