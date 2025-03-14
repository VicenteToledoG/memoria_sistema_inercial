function [accelGlobal, velGlobal, posGlobal] = transformAccelToGlobal(eulerAngles, accelData, dt, prevVelGlobal, prevPosGlobal)
    % Entrada:
    % eulerAngles  -> Vector [yaw, pitch, roll] en radianes.
    % accelData    -> Vector 1x3 con aceleración en el marco del sensor [ax, ay, az].
    % dt           -> Paso de tiempo (delta t) entre iteraciones.
    % prevVelGlobal-> Velocidad global del paso anterior.
    % prevPosGlobal-> Posición global del paso anterior.
    
    % Salida:
    % accelGlobal  -> Aceleración transformada al marco global.
    % velGlobal    -> Velocidad global.
    % posGlobal    -> Posición global.

    % Descomposición de los ángulos de Euler
    yaw = -eulerAngles(1);   % Rotación alrededor del eje Z (Yaw)
    pitch = eulerAngles(2); % Inclinación sobre el eje X local (Pitch)
    roll = eulerAngles(3);  % Rotación sobre el eje X local (Roll)

    % Matrices de rotación
    Rz = [cos(yaw), -sin(yaw), 0; 
          sin(yaw),  cos(yaw), 0;
          0,         0,        1];  % Rotación alrededor del eje Z (Yaw)

    Ry = [cos(pitch), 0, sin(pitch);
          0,          1, 0;
         -sin(pitch), 0, cos(pitch)];  % Rotación alrededor del eje Y (Pitch)

    Rx = [1, 0,          0;
          0, cos(roll), -sin(roll);
          0, sin(roll),  cos(roll)];  % Rotación alrededor del eje X (Roll)

    % Matriz de rotación completa (local a global)
    R = Rz * Ry * Rx;

    % Transformación de la aceleración del marco del sensor al marco global
    accelGlobal = (R * accelData.').';  % Vector transformado
    %accelGlobal(1)=2*accelGlobal(1);
    %accelGlobal(2)=2*accelGlobal(2);

    gravity = [0 0 -9.81];
    globalAcc = (accelGlobal - gravity); 
    globalAcc(1)=-globalAcc(1);
    globalAcc(2)=-globalAcc(2);
    % Cálculo de la velocidad global
    if isempty(prevVelGlobal)
        % Si es la primera iteración, inicializamos la velocidad
        velGlobal = globalAcc * dt;
    else
        % Integra la aceleración global para obtener la velocidad
        velGlobal = prevVelGlobal + globalAcc * dt;
    end

    % Cálculo de la posición global
    if isempty(prevPosGlobal)
        % Si es la primera iteración, inicializamos la posición
        posGlobal = velGlobal * dt;
    else
        % Integra la velocidad global para obtener la posición
        posGlobal = prevPosGlobal + velGlobal * dt;
    end
end