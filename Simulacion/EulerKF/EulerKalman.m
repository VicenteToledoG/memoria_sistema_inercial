function [phi, theta, psi] = EulerKalman(A, z, phi_i, theta_i, psi_i)
persistent H Q R
persistent x P
persistent firstRun
persistent last_valid_state
persistent transition_region

if isempty(firstRun)
    H = eye(4);
    Q = diag([0.001, 0.001, 0.001, 0.001]);
    R = diag([0.1, 0.1, 0.1, 1.0]);  % Mayor incertidumbre en la última componente
    
    % Inicialización del cuaternión como vector columna
    x = eul2quat([phi_i theta_i psi_i], 'ZYX')';
    x = x(:);
    P = 0.1 * eye(4);
    
    % Inicializar el estado válido anterior
    last_valid_state = x;
    
    % Región de transición para el manejo suave cerca de los polos
    transition_region = struct('start', 80*pi/180, 'end', 89*pi/180);
    
    firstRun = 1;
end

% Predicción
xp = A * x;
xp = xp / norm(xp);

% Obtener ángulos de Euler de la predicción para análisis
pred_euler = quat2eul(xp', 'ZYX');
pred_theta = pred_euler(2);

% Corrección
z = z(:);
z_norm = z / norm(z);

% Verificar si estamos cerca de un polo
near_pole = abs(pred_theta) > transition_region.start;

if near_pole
    % Calcular factor de mezcla basado en la proximidad al polo
    blend_factor = (abs(pred_theta) - transition_region.start) / ...
                  (transition_region.end - transition_region.start);
    blend_factor = min(max(blend_factor, 0), 1);
    
    % Ajustar matrices de covarianza para dar más peso a la predicción del giroscopio
    R_adjusted = R * (1 + blend_factor * 10);  % Aumentar incertidumbre en medición
    Q_adjusted = Q * (1 - blend_factor * 0.5); % Reducir incertidumbre en predicción
else
    R_adjusted = R;
    Q_adjusted = Q;
end

% Propagar covarianza con matrices ajustadas
Pp = A * P * A' + Q_adjusted;
K = Pp * H' / (H * Pp * H' + R_adjusted);

% Manejar la ambigüedad del cuaternión
if dot(xp, z_norm) < 0
    z_norm = -z_norm;
end

% Innovación y actualización
innovation = z_norm - xp;

if near_pole
    % Reducir la influencia de la innovación en componentes que afectan al yaw
    innovation = innovation .* (1 - blend_factor);
end

x = xp + K * innovation;
x = x / norm(x);

% Actualización de la covarianza
I = eye(4);
P = (I - K*H) * Pp * (I - K*H)' + K*R_adjusted*K';

% Actualizar el último estado válido cuando no estamos muy cerca del polo
if abs(pred_theta) < transition_region.end
    last_valid_state = x;
end

% Conversión final a ángulos de Euler
eul = quat2eul(x', 'ZYX');

% Aplicar límites y wrap a los ángulos
phi = wrapToPi(eul(1));
theta = wrapToPi(eul(2));
psi = wrapToPi(eul(3));

end

