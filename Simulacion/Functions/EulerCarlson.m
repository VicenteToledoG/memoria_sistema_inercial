function [phi, theta, psi] = EulerCarlson(z, rates, dt, phi_i, theta_i, psi_i)
    persistent x Cxx firstRun
    persistent H Q R P
    if isempty(firstRun)
        H = [1 0 0; 0 1 0];
        Q = diag([0.0001, 0.0001, 0.1]);
        R = diag([6, 6]);
        x = [phi_i theta_i psi_i]';
        P = 10*eye(3);
        Cxx = chol(P)';
        firstRun = 1;
    end
    
    % Prediction
    A = Ajacob(x, rates, dt);
    xp = fx(x, rates, dt);
    xp = adjustEulerAngles(xp);
    
    % Carlson update
    [x, Cxx] = kalman_carlson(z, R, H, xp, Cxx);
    
    % Convert to Euler angles
    eul = [x(1) x(2) x(3)];
    rotmZYX = eul2rotm(eul);
    eul = rotm2eul(rotmZYX);
    phi = eul(1);
    theta = eul(2);
    psi = eul(3);
end


% Common state transition functions
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

function A = Ajacob(xhat, rates, dt)
    A = zeros(3, 3);
    phi = xhat(1);
    theta = xhat(2);
    p = rates(1);
    q = rates(2);
    r = rates(3);
    
    A(1,1) = q*cos(phi)*tan(theta) - r*sin(phi)*tan(theta);
    A(1,2) = q*sin(phi)*sec(theta)^2 + r*cos(phi)*sec(theta)^2;
    A(1,3) = 0;
    A(2,1) = -q*sin(phi) - r*cos(phi);
    A(2,2) = 0;
    A(2,3) = 0;
    A(3,1) = q*cos(phi)*sec(theta) - r*sin(phi)*sec(theta);
    A(3,2) = q*sin(phi)*sec(theta)*tan(theta) + r*cos(phi)*sec(theta)*tan(theta);
    A(3,3) = 0;
    
    A = eye(3) + A*dt;
end