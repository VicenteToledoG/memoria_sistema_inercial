function eul = rotm2eul(R)
    % Convierte matriz de rotación a ángulos de Euler en secuencia ZYX
    % Input:
    %   R: matriz de rotación 3x3
    % Output:
    %   eul: vector [phi theta psi] en radianes
    
    % Extraer ángulos de Euler
    % Manejo especial para el caso de gimbal lock
    if abs(R(1,3)) >= 1 - eps
        % Gimbal lock en theta = ±90°
        psi = 0; % Yaw
        if R(1,3) < 0
            theta = pi/2; % Pitch
            phi = psi + atan2(R(2,1), R(2,2)); % Roll
        else
            theta = -pi/2;
            phi = -psi + atan2(-R(2,1), -R(2,2));
        end
    else
        theta = -asin(R(1,3)); % Pitch
        phi = atan2(R(2,3)/cos(theta), R(3,3)/cos(theta)); % Roll
        psi = atan2(R(1,2)/cos(theta), R(1,1)/cos(theta)); % Yaw
    end
    
    eul = [phi theta psi];
end