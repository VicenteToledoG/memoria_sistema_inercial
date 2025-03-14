function xp = fx(xhat, rates, dt)
    phi = xhat(1);
    theta = xhat(2);
    p = rates(1);
    q = rates(2);
    r = rates(3);
    
    xdot = zeros(3, 1);
    xdot(1) = p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
    xdot(2) = q*cos(phi) - r*sin(phi);
    xdot(3) = q*sin(phi)*sec(theta) + r*cos(phi)*sec(theta);
    
    xp = xhat + xdot*dt;
end