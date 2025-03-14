% Takasu Filter implementation for AHRS
function [phi, theta, psi] = EulerTakasu(accelData, rates, dt, phi_i, theta_i, psi_i)
    persistent x P firstRun
    persistent H R
    if isempty(firstRun)
        H = [1 0 0; 0 1 0];
        R = diag([6, 6]);
        x = [phi_i theta_i psi_i]';
        P = 10*eye(3);
        firstRun = 1;
    end


    [phi_a, theta_a] = EulerAccel(accelData(1), accelData(2), accelData(3), 0, 0, 0);
    z=[phi_a, theta_a]';

    xp = fx(x, rates, dt);
    xp = adjustEulerAngles(xp);
    
    % Takasu update
    dz = z - H*xp;
    dz = adjustEulerAngles(dz);
    [x, P] = kalman_takasu(xp, P, dz, R, H);
    
    % Convert to Euler angles
    eul = [x(1) x(2) x(3)];
    rotmZYX = eul2rotm(eul);
    eul = rotm2eul(rotmZYX);  %esto elimina el gimbal lock
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
