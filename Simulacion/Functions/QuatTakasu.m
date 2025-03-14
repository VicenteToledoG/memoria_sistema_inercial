function q_out = QuatTakasu(z, rates, dt, phi_i, theta_i, psi_i)
    persistent x P firstRun
    persistent H R F
    persistent Kp
    
    if isempty(firstRun)
        H = eye(4);
        P = eye(4);
        R = diag([10, 10, 10, 10]);
        x = euler_to_quaternion(phi_i, theta_i, psi_i);
        x = normalize_quaternion(x);
        Kp = 1; % Ajustar según sea necesario
        firstRun = 1;
    end
    
    % Predicción del estado usando Kalman
    F = gyro_transition_matrix(rates(1), rates(2), rates(3), dt);
    x = F * x;
    x = normalize_quaternion(x);
    
    % Actualización de la covarianza
    P = F * P * F';
    
    % Medición en cuaterniones
    [phi_a, theta_a, ~] = EulerAccel(z(1), z(2), z(3), 0, 0, 0);
    z_quat = euler_to_quaternion(phi_a, theta_a, 0);
    z_quat = normalize_quaternion(z_quat);
    
    % Calcular innovación
    dz = quaternion_difference(z_quat, H * x);
    
    % Actualización Takasu
    [x, P, ~] = kalman_takasu(x, P, dz, R, H);
    x = normalize_quaternion(x);
    
    % Aplicar corrección PI después de la estimación Kalman
    % Convertir acelerómetro a unidades g y normalizar
    accel_normalized = z(1:3) / 9.81; % Convertir de m/s^2 a g
    accel_normalized = accel_normalized / norm(accel_normalized);
    
    % Calcular dirección estimada de la gravedad desde el cuaternión actual
    v = [2*(x(2)*x(4) - x(1)*x(3));
         2*(x(1)*x(2) + x(3)*x(4));
         x(1)^2 - x(2)^2 - x(3)^2 + x(4)^2];
    
    % Calcular error y actualizar integral
    error = cross(v, accel_normalized');
    
    % Aplicar corrección PI
    correction = -Kp * error;
    
    % Calcular la tasa de cambio del cuaternión debido a la corrección
    pDot = 0.5 * quaternion_multiply(x, [0; correction(1); correction(2); correction(3)]);
    
    % Integrar la tasa de cambio
    x = x + pDot * dt;
    x = normalize_quaternion(x);
    
    % Convertir a objeto quaternion de MATLAB
    q_out = quaternion(x(1), x(2), x(3), x(4));
end

function q = quaternion_multiply(q1, q2)
    w1 = q1(1); x1 = q1(2); y1 = q1(3); z1 = q1(4);
    w2 = q2(1); x2 = q2(2); y2 = q2(3); z2 = q2(4);
    
    q = [w1*w2 - x1*x2 - y1*y2 - z1*z2;
         w1*x2 + x1*w2 + y1*z2 - z1*y2;
         w1*y2 - x1*z2 + y1*w2 + z1*x2;
         w1*z2 + x1*y2 - y1*x2 + z1*w2];
end

function q = euler_to_quaternion(roll, pitch, yaw)
    % Mitad de los ángulos
    cr = cos(roll * 0.5);
    sr = sin(roll * 0.5);
    cp = cos(pitch * 0.5);
    sp = sin(pitch * 0.5);
    cy = cos(yaw * 0.5);
    sy = sin(yaw * 0.5);
    
    % Cálculo del cuaternión [w x y z]
    q = [cr*cp*cy + sr*sp*sy;
         sr*cp*cy - cr*sp*sy;
         cr*sp*cy + sr*cp*sy;
         cr*cp*sy - sr*sp*cy];
end

function F = gyro_transition_matrix(wx, wy, wz, dt)
    w = [wx; wy; wz];
    norm_w = norm(w);
    
    if norm_w < 1e-10
        F = eye(4);
        return;
    end
    
    angle = norm_w * dt * 0.5;
    axis = w / norm_w;
    
    s = sin(angle);
    c = cos(angle);
    
    x = axis(1);
    y = axis(2);
    z = axis(3);
    
    F = [   c,  -x*s,  -y*s,  -z*s;
         x*s,     c,  z*s,  -y*s;
         y*s,  -z*s,     c,   x*s;
         z*s,   y*s,  -x*s,     c];
end

function q_norm = normalize_quaternion(q)
    norm_q = norm(q);
    if norm_q < eps
        q_norm = [1; 0; 0; 0];
    else
        q_norm = q / norm_q;
    end
end

function dq = quaternion_difference(q1, q2)
    q1 = normalize_quaternion(q1);
    q2 = normalize_quaternion(q2);
    
    % Asegurar que estamos tomando el camino más corto
    if q1'*q2 < 0
        q2 = -q2;
    end
    
    dq = q1 - q2;
    dq = normalize_quaternion(dq);
end
