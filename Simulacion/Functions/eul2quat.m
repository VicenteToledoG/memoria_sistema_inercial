function q = eul2quat(euler_angles, sequence)
    % Convierte ángulos de Euler a cuaterniones
    % Input:
    %   euler_angles: vector [phi theta psi] en radianes
    %   sequence: string que indica la secuencia de rotación (e.g., 'ZYX')
    % Output:
    %   q: cuaternión en formato [w x y z]
    
    % Extraer ángulos
    phi = euler_angles(1);    % roll
    theta = euler_angles(2);  % pitch
    psi = euler_angles(3);    % yaw
    
    % Calcular senos y cosenos
    cy = cos(psi * 0.5);
    sy = sin(psi * 0.5);
    cp = cos(theta * 0.5);
    sp = sin(theta * 0.5);
    cr = cos(phi * 0.5);
    sr = sin(phi * 0.5);
    
    if strcmpi(sequence, 'ZYX')
        % Calcular componentes del cuaternión para secuencia ZYX
        q = zeros(4,1);
        q(1) = cr * cp * cy + sr * sp * sy;  % w
        q(2) = sr * cp * cy - cr * sp * sy;  % x
        q(3) = cr * sp * cy + sr * cp * sy;  % y
        q(4) = cr * cp * sy - sr * sp * cy;  % z
    else
        error('Secuencia de rotación no soportada. Solo se admite ZYX.');
    end
end