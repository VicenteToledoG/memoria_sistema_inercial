function R = eul2rotm(eul)
    % Convierte ángulos de Euler a matriz de rotación en secuencia ZYX
    % Input: 
    %   eul: vector [phi theta psi] en radianes
    % Output:
    %   R: matriz de rotación 3x3
    
    phi = eul(1);   % roll
    theta = eul(2); % pitch
    psi = eul(3);   % yaw
    
    % Matriz de rotación para roll (X)
    Rx = [1 0 0;
          0 cos(phi) -sin(phi);
          0 sin(phi) cos(phi)];
    
    % Matriz de rotación para pitch (Y)
    Ry = [cos(theta) 0 sin(theta);
          0 1 0;
          -sin(theta) 0 cos(theta)];
    
    % Matriz de rotación para yaw (Z)
    Rz = [cos(psi) -sin(psi) 0;
          sin(psi) cos(psi) 0;
          0 0 1];
    
    % Matriz de rotación completa ZYX
    R = Rz * Ry * Rx;
end