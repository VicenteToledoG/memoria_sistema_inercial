function visualizeOrientations(quatEst, quatEKF, quatGyr, quatKF, quatUDU, quatTakasu, quatCarlson, quatTakasu_mejorado,quatGroundTruth)
    % Crear visualizador
    viewer = MultiOrientationViewer(9);  % Cambiado de 7 a 8
    
    % Obtener número de muestras
    numSamples = length(quatEst);
    
    % Índices decimados
    decimatedIndices = 1:viewer.DecimationFactor:numSamples;
    
    % Preasignar array de quaterniones
    currentQuats = quaternion.empty();
    currentQuats(9) = quaternion(1,0,0,0);  % Cambiado de 7 a 8
    
    % Tiempo entre actualizaciones
    updatePeriod = viewer.DecimationFactor/2500;
    
    % Deshabilitar advertencias temporalmente
    warning('off', 'MATLAB:handle_graphics:exceptions:SceneNode');
    
    tic;
    % Loop de animación
    for idx = decimatedIndices
        % Actualizar quaterniones
        currentQuats(1) = quatEst(idx);
        currentQuats(2) = quatEKF(idx);
        currentQuats(3) = quatGyr(idx);
        currentQuats(4) = quatKF(idx);
        currentQuats(5) = quatUDU(idx);
        currentQuats(6) = quatTakasu(idx);
        currentQuats(7) = quatCarlson(idx);
        currentQuats(8) = quatTakasu_mejorado(idx);  % Agregada nueva línea
        currentQuats(9) = quatGroundTruth(idx);  % Agregada nueva línea
        
        % Actualizar visualización
        viewer.update(currentQuats);
        
        % Esperar solo si es necesario
        elapsed = toc;
        if elapsed < updatePeriod
            pause(updatePeriod - elapsed);
        end
        tic;
    end
    
    % Restaurar advertencias
    warning('on', 'MATLAB:handle_graphics:exceptions:SceneNode');
end