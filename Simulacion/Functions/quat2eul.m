function euler = quat2eul(q, sequence)
    % Convierte un cuaternión a ángulos de Euler
    % Input:
    %   q: cuaternión en formato [w x y z]
    %   sequence: string que indica la secuencia de rotación (e.g., 'ZYX')
    % Output:
    %   euler: vector [phi theta psi] en radianes
    
    % Asegurarse de que el cuaternión esté en la orientación correcta
    if size(q,1) > size(q,2)
        q = q';
    end
    
    % Extraer componentes del cuaternión
    qw = q(1);
    qx = q(2);
    qy = q(3);
    qz = q(4);
    
    if strcmpi(sequence, 'ZYX')
        % Calcular ángulos de Euler para secuencia ZYX (roll, pitch, yaw)
        
        % Roll (phi)
        sinr_cosp = 2 * (qw * qx + qy * qz);
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
        phi = atan2(sinr_cosp, cosr_cosp);
        
        % Pitch (theta)
        sinp = 2 * (qw * qy - qz * qx);
        if abs(sinp) >= 1
            theta = copysign(pi/2, sinp); % Usar pi/2 si sinp = 1, -pi/2 si sinp = -1
        else
            theta = asin(sinp);
        end
        
        % Yaw (psi)
        siny_cosp = 2 * (qw * qz + qx * qy);
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
        psi = atan2(siny_cosp, cosy_cosp);
        
        euler = [phi theta psi];
    else
        error('Secuencia de rotación no soportada. Solo se admite ZYX.');
    end
end

function y = copysign(x, y)
    % Función auxiliar para emular la función copysign de C++
    % Retorna x con el signo de y
    if y >= 0
        y = abs(x);
    else
        y = -abs(x);
    end
end