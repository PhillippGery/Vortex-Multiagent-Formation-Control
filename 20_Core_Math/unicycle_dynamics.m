function dq = unicycle_dynamics(q, u_v, l, v_max, w_max)
    % q: Current state [x; y; theta]
    % u_v: Desired velocity of the look-ahead point [vx_des; vy_des]
    
    th = q(3);
    
    % 1. The Feedback Linearization Matrix Inverse
    M_inv = [ cos(th),          sin(th); 
             -(1/l)*sin(th),    (1/l)*cos(th)];
             
    % 2. Calculate physical inputs (v, omega)
    vw = M_inv * u_v;
    v_cmd = vw(1);
    w_cmd = vw(2);
    
    % 3. Apply Hard Saturation Limits (Crucial for realistic simulation)
    v_actual = max(min(v_cmd, v_max), -v_max);
    w_actual = max(min(w_cmd, w_max), -w_max);
    
    % 4. Unicycle Kinematics
    dx = v_actual * cos(th);
    dy = v_actual * sin(th);
    dth = w_actual;
    
    dq = [dx; dy; dth];
end