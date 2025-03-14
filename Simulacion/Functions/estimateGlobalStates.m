function [globalAcc, globalVel, globalPos] = estimateGlobalStates(accelLocal, quatEKF, globalVelPrev, globalPosPrev, dt)
    % Entrada:
    % accelLocal: Aceleración medida en el marco del sensor (local)
    % quatEKF: Cuaternion de la orientación estimada (EKF)
    % globalVelPrev: Velocidad global en la iteración previa
    % globalPosPrev: Posición global en la iteración previa
    % dt: Tiempo entre muestras (1 / imuFs)
    
    % Rotar aceleración local al marco global
    rotationMatrix = rotmat(quatEKF, 'frame');  % Matriz de rotación desde local a global
    globalAcc = rotationMatrix * accelLocal'; % Transformar aceleración a global

    % Restar el vector gravedad (aproximado a 9.81 m/s^2 en el eje Z)
    gravity = [0; 9.81; 0];
    globalAcc = (globalAcc - gravity)';

    % Integrar para obtener velocidad global
    globalVel = globalVelPrev + globalAcc .* dt; % Transpuesta para volver a fila
    
    % Integrar para obtener posición global
    globalPos = globalPosPrev + globalVel .* dt;
end
