%% Análisis de Filtros de Fusión Sensorial con Datos Reales de IMU
% Este script evalúa y compara diferentes algoritmos de fusión sensorial utilizando 
% datos reales capturados de una IMU (Unidad de Medición Inercial).
%
% Los filtros implementados incluyen:
% - Filtro de Referencia de MATLAB (insfilterNonholonomic)
% - Filtro de Kalman Extendido (EKF)
% - Integración de Giroscopio (Gyro)
% - Filtro de Kalman Lineal (KF)
% - Filtro UDU
% - Filtro Takasu
% - Filtro Carlson
% - Filtro Takasu Mejorado
%
% El script:
%   1. Carga los datos reales de la IMU desde un archivo .mat
%   2. Convierte los datos raw del sensor a unidades físicas
%   3. Ejecuta cada uno de los filtros con los datos reales
%   4. Visualiza los resultados mediante gráficos comparativos
%   5. Calcula y muestra los tiempos de ejecución de cada filtro

%% 1. Inicialización y configuración de rutas
% Esta sección configura las rutas de directorios necesarias para acceder
% a las funciones de los diferentes filtros

% Agregar directorios necesarios al path
addpath('GyroscopeIntegration');    % Funciones para integración del giroscopio
addpath('AccelerometerMagnetometer'); % Funciones para acelerómetro/magnetómetro
addpath('EulerKF');                 % Filtro de Kalman para ángulos de Euler
addpath('EulerEKF');                % Filtro de Kalman Extendido para ángulos de Euler
addpath('EulerUKF');                % Filtro de Kalman Unscented para ángulos de Euler
addpath('Functions');               % Funciones auxiliares generales

%% 2. Carga de datos reales de la IMU
% Carga los datos registrados por la IMU desde archivos .mat
% Seleccione el conjunto de datos a utilizar descomentando la carga requerida

load("Rotacion_3.mat");  % Carga datos de rotación

%load("Locura.mat");      % Datos alternativos (descomentado según necesidad)

%load("prueba_dibujo.mat"); % Datos alternativos (descomentado según necesidad)

%% 3. Configuración de parámetros iniciales
% Define los parámetros básicos para la simulación y configura el filtro de referencia

numsamples = 25000;      % Número de muestras a procesar
fs = 2500;               % Frecuencia de muestreo en Hz

% Inicializar filtro de referencia (Matlab)
gndFusion = insfilterNonholonomic('ReferenceFrame', 'ENU', ...
    'IMUSampleRate', fs, ...
    'DecimationFactor', 2);

% Configuración de parámetros del filtro de referencia
gndFusion.ZeroVelocityConstraintNoise = 1e-2;    % Ruido en restricción de velocidad cero
gndFusion.GyroscopeNoise = 4e-6;                 % Ruido del giroscopio
gndFusion.GyroscopeBiasNoise = 4e-14;            % Ruido del bias del giroscopio
gndFusion.AccelerometerNoise = 4.8e-2;           % Ruido del acelerómetro
gndFusion.AccelerometerBiasNoise = 4e-14;        % Ruido del bias del acelerómetro
gndFusion.StateCovariance = 1e-9*eye(16);        % Covarianza de estado inicial

%% 4. Inicialización de variables para almacenar resultados
% Prepara arrays para guardar las estimaciones y resultados de cada filtro

% Filtro de Referencia de MATLAB
estPosition = zeros(numsamples, 3);              % Posición estimada
estOrientation = quaternion.zeros(numsamples, 1); % Orientación estimada (cuaterniones)
globalAccMatlab = zeros(numsamples, 3);          % Aceleración global estimada
globalVelMatlab = zeros(numsamples, 3);          % Velocidad global estimada
globalPosMatlab = zeros(numsamples, 3);          % Posición global estimada
eulerEst = zeros(numsamples, 3);                 % Ángulos de Euler estimados
timeRef = zeros(numsamples, 1);                  % Tiempo de ejecución

% Filtro EKF (Extended Kalman Filter)
eulerEKF = zeros(numsamples, 3);                 % Ángulos de Euler estimados
globalAccEKF = zeros(numsamples, 3);             % Aceleración global estimada
globalVelEKF = zeros(numsamples, 3);             % Velocidad global estimada
globalPosEKF = zeros(numsamples, 3);             % Posición global estimada
timeEKF = zeros(numsamples, 1);                  % Tiempo de ejecución
quatEKF = quaternion.zeros(numsamples, 1);       % Orientación estimada (cuaterniones)

% Integración de Giroscopio
eulerGyr = zeros(numsamples, 3);                 % Ángulos de Euler estimados
globalAccGyr = zeros(numsamples, 3);             % Aceleración global estimada
globalVelGyr = zeros(numsamples, 3);             % Velocidad global estimada
globalPosGyr = zeros(numsamples, 3);             % Posición global estimada
timeGyr = zeros(numsamples, 1);                  % Tiempo de ejecución
quatGyr = quaternion.zeros(numsamples, 1);       % Orientación estimada (cuaterniones)

% Filtro de Kalman Lineal
eulerKF = zeros(numsamples, 3);                  % Ángulos de Euler estimados
globalAccKF = zeros(numsamples, 3);              % Aceleración global estimada
globalVelKF = zeros(numsamples, 3);              % Velocidad global estimada
globalPosKF = zeros(numsamples, 3);              % Posición global estimada
timeKF = zeros(numsamples, 1);                   % Tiempo de ejecución
quatKF = quaternion.zeros(numsamples, 1);        % Orientación estimada (cuaterniones)

% Filtro UDU
eulerUDU = zeros(numsamples, 3);                 % Ángulos de Euler estimados
globalAccUDU = zeros(numsamples, 3);             % Aceleración global estimada
globalVelUDU = zeros(numsamples, 3);             % Velocidad global estimada
globalPosUDU = zeros(numsamples, 3);             % Posición global estimada
timeUDU = zeros(numsamples, 1);                  % Tiempo de ejecución
quatUDU = quaternion.zeros(numsamples, 1);       % Orientación estimada (cuaterniones)

% Filtro Takasu
eulerTakasu = zeros(numsamples, 3);              % Ángulos de Euler estimados
eulerTakasu2 = zeros(numsamples, 3);             % Ángulos de Euler adicionales
globalAccTakasu = zeros(numsamples, 3);          % Aceleración global estimada
globalVelTakasu = zeros(numsamples, 3);          % Velocidad global estimada
globalPosTakasu = zeros(numsamples, 3);          % Posición global estimada
timeTakasu = zeros(numsamples, 1);               % Tiempo de ejecución
quatTakasu = quaternion.zeros(numsamples, 1);    % Orientación estimada (cuaterniones)

% Filtro Carlson
eulerCarlson = zeros(numsamples, 3);             % Ángulos de Euler estimados
globalAccCarlson = zeros(numsamples, 3);         % Aceleración global estimada
globalVelCarlson = zeros(numsamples, 3);         % Velocidad global estimada
globalPosCarlson = zeros(numsamples, 3);         % Posición global estimada
timeCarlson = zeros(numsamples, 1);              % Tiempo de ejecución
quatCarlson = quaternion.zeros(numsamples, 1);   % Orientación estimada (cuaterniones)

% Filtro Takasu Mejorado
eulerTakasu_mejorado = zeros(numsamples, 3);     % Ángulos de Euler estimados
globalAccTakasu_mejorado  = zeros(numsamples, 3); % Aceleración global estimada
globalVelTakasu_mejorado  = zeros(numsamples, 3); % Velocidad global estimada
globalPosTakasu_mejorado  = zeros(numsamples, 3); % Posición global estimada
timeTakasu_mejorado  = zeros(numsamples, 1);     % Tiempo de ejecución
quatTakasu_mejorado  = quaternion.zeros(numsamples, 1); % Orientación estimada (cuaterniones)

% Variables para velocidades y posiciones previas (necesarias para integración)
velPrevUDU = [0, 0, 0];                          % Velocidad previa UDU
posPrevUDU = [0, 0, 0];                          % Posición previa UDU
velPrevTakasu = [0, 0, 0];                       % Velocidad previa Takasu
posPrevTakasu = [0, 0, 0];                       % Posición previa Takasu
velPrevCarlson = [0, 0, 0];                      % Velocidad previa Carlson
posPrevCarlson = [0, 0, 0];                      % Posición previa Carlson
velPrevMatlab = [0, 0, 0];                       % Velocidad previa Matlab
posPrevMatlab = [0, 0, 0];                       % Posición previa Matlab
velPrevEKF = [0, 0, 0];                          % Velocidad previa EKF
posPrevEKF = [0, 0, 0];                          % Posición previa EKF
velPrevGyr = [0, 0, 0];                          % Velocidad previa Gyro
posPrevGyr = [0, 0, 0];                          % Posición previa Gyro
velPrevKF = [0, 0, 0];                           % Velocidad previa KF
posPrevKF = [0, 0, 0];                           % Posición previa KF
velPrevTakasu_mejorado = [0, 0, 0];              % Velocidad previa Takasu Mejorado
posPrevTakasu_mejorado = [0, 0, 0];              % Posición previa Takasu Mejorado

%% 5. Conversión de datos raw del sensor a unidades físicas
% Los datos crudos del sensor (raw) son convertidos a unidades físicas calibradas
% utilizando los rangos y centros determinados previamente para cada eje

% Acelerómetro - Conversión a m/s²
g = 9.81;  % Aceleración debido a la gravedad (m/s²)

% Conversión aceleración eje X
max_X_raw = 17000; min_X_raw = -16400; center_X = 0;  % Valores de calibración
accX = zeros(size(accX_raw));                         % Inicializa array de salida
% Aplicar escala y offset según el valor respecto al centro
accX(accX_raw >= center_X) = (accX_raw(accX_raw >= center_X) - center_X) * (g / (max_X_raw - center_X));
accX(accX_raw < center_X) = (accX_raw(accX_raw < center_X) - center_X) * (g / (center_X - min_X_raw));

% Conversión aceleración eje Y
max_Y_raw = 16400; min_Y_raw = -16500; center_Y = 0;  % Valores de calibración
accY = zeros(size(accY_raw));                         % Inicializa array de salida
% Aplicar escala y offset según el valor respecto al centro
accY(accY_raw >= center_Y) = (accY_raw(accY_raw >= center_Y) - center_Y) * (g / (max_Y_raw - center_Y));
accY(accY_raw < center_Y) = (accY_raw(accY_raw < center_Y) - center_Y) * (g / (center_Y - min_Y_raw));

% Conversión aceleración eje Z
max_Z_raw = 19350; min_Z_raw = -14600; center_Z = 0;  % Valores de calibración
accZ = zeros(size(accZ_raw));                         % Inicializa array de salida
% Aplicar escala y offset según el valor respecto al centro
accZ(accZ_raw >= center_Z) = (accZ_raw(accZ_raw >= center_Z) - center_Z) * (g / (max_Z_raw - center_Z));
accZ(accZ_raw < center_Z) = (accZ_raw(accZ_raw < center_Z) - center_Z) * (g / (center_Z - min_Z_raw));

% Giroscopio - Conversión a rad/s (aquí se usa un factor unitario, luego se escala)
g = 1;  % Factor de escala unitario, se ajusta posteriormente

% Conversión velocidad angular eje X
max_X_raw = 7600; min_X_raw = -6500; center_X = 625;  % Valores de calibración
gyrX = zeros(size(gyrX_raw));                         % Inicializa array de salida
% Aplicar escala y offset según el valor respecto al centro
gyrX(gyrX_raw >= center_X) = (gyrX_raw(gyrX_raw >= center_X) - center_X) * (g / (max_X_raw - center_X));
gyrX(gyrX_raw < center_X) = (gyrX_raw(gyrX_raw < center_X) - center_X) * (g / (center_X - min_X_raw));

% Conversión velocidad angular eje Y
max_Y_raw = 6500; min_Y_raw = -7600; center_Y = -463; % Valores de calibración
gyrY = zeros(size(gyrY_raw));                         % Inicializa array de salida
% Aplicar escala y offset según el valor respecto al centro
gyrY(gyrY_raw >= center_Y) = (gyrY_raw(gyrY_raw >= center_Y) - center_Y) * (g / (max_Y_raw - center_Y));
gyrY(gyrY_raw < center_Y) = (gyrY_raw(gyrY_raw < center_Y) - center_Y) * (g / (center_Y - min_Y_raw));

% Conversión velocidad angular eje Z
max_Z_raw = 7600; min_Z_raw = -7600; center_Z = 200;  % Valores de calibración
gyrZ = zeros(size(gyrZ_raw));                         % Inicializa array de salida
% Aplicar escala y offset según el valor respecto al centro
gyrZ(gyrZ_raw >= center_Z) = (gyrZ_raw(gyrZ_raw >= center_Z) - center_Z) * (g / (max_Z_raw - center_Z));
gyrZ(gyrZ_raw < center_Z) = (gyrZ_raw(gyrZ_raw < center_Z) - center_Z) * (g / (center_Z - min_Z_raw));

%% 6. Loop principal de procesamiento
% Ejecuta cada uno de los filtros para cada muestra de datos
% y almacena los resultados para su posterior análisis

fs = 2500;    % Frecuencia de muestreo en Hz
dt = 1/fs;    % Intervalo de tiempo entre muestras

% Limpiar instancias previas de los objetos de los filtros
clear EulerCarlson;
clear EulerTakasu;
clear EulerUDU;
clear EulerKalman;
clear EulerGyro;
clear EulerEKF;
clear EulerTakasu_mejorado;

for idx = 1:numsamples
    % Preparar datos del sensor para esta muestra
    accelData = -[accX(idx), accY(idx), accZ(idx)];             % Datos de aceleración
    gyroData = 2*[gyrX(idx), gyrY(idx), gyrZ(idx)];             % Datos de giroscopio (factor 2 para escalar)
    [phi_a(idx), theta_a(idx)] = EulerAccel(accX(idx), accY(idx), accZ(idx), 0, 0, 0); % Ángulos de acelerómetro
    
    % 1. Filtro de referencia Matlab
    tic;  % Inicio medición de tiempo
    predict(gndFusion, accelData, gyroData);  % Actualiza el filtro con mediciones actuales
    [estPosition(idx,:), estOrientation(idx,:)] = pose(gndFusion);  % Obtiene posición y orientación
    eulerEst(idx, :) = euler(estOrientation(idx,:), 'ZYX', 'frame');  % Convierte a ángulos de Euler
    
    % Transforma aceleración al marco global y calcula velocidad/posición por integración
    [globalAccMatlab(idx,:), globalVelMatlab(idx,:), globalPosMatlab(idx,:)] = ...
        transformAccelToGlobal(eulerEst(idx,:), accelData, dt, velPrevMatlab, posPrevMatlab);
    
    timeRef(idx) = toc;  % Fin medición de tiempo
    
    % Actualiza velocidad y posición previas para la siguiente iteración
    velPrevMatlab = globalVelMatlab(idx,:);
    posPrevMatlab = globalPosMatlab(idx,:);
    
    % 2. Filtro EKF (Extended Kalman Filter)
    tic;  % Inicio medición de tiempo
    
    % Aplica el filtro EKF para estimar orientación
    [phi, theta, psi] = EulerEKF([phi_a(idx) theta_a(idx)]', gyroData, dt, -phi_a(idx), -theta_a(idx), 0);
    
    % Convierte a cuaternión y luego a ángulos de Euler para almacenar
    quatEKF(idx) = quaternion([-phi, -theta, -psi], 'euler', 'XYZ', 'frame');
    eulerEKF(idx, :) = euler(quatEKF(idx), 'ZYX', 'frame');
    
    % Transforma aceleración al marco global y calcula velocidad/posición por integración
    [globalAccEKF(idx,:), globalVelEKF(idx,:), globalPosEKF(idx,:)] = ...
        transformAccelToGlobal(eulerEKF(idx,:), accelData, dt, velPrevEKF, posPrevEKF);
    
    timeEKF(idx) = toc;  % Fin medición de tiempo
    
    % Actualiza velocidad y posición previas para la siguiente iteración
    velPrevEKF = globalVelEKF(idx,:);
    posPrevEKF = globalPosEKF(idx,:);
    
    % 3. Integración del Giroscopio
    tic;  % Inicio medición de tiempo
    
    % Integra las velocidades angulares para estimar orientación
    [phi2, theta2, psi2] = EulerGyro(-gyroData(1), -gyroData(2), -gyroData(3), dt, -phi_a(idx), -theta_a(idx), 0);
    
    % Convierte a cuaternión y luego a ángulos de Euler para almacenar
    quatGyr(idx) = quaternion([-phi2, -theta2, -psi2], 'euler', 'XYZ', 'frame');
    eulerGyr(idx, :) = euler(quatGyr(idx), 'ZYX', 'frame');
    
    % Transforma aceleración al marco global y calcula velocidad/posición por integración
    [globalAccGyr(idx,:), globalVelGyr(idx,:), globalPosGyr(idx,:)] = ...
        transformAccelToGlobal(eulerGyr(idx,:), accelData, dt, velPrevGyr, posPrevGyr);
    
    timeGyr(idx) = toc;  % Fin medición de tiempo
    
    % Actualiza velocidad y posición previas para la siguiente iteración
    velPrevGyr = globalVelGyr(idx,:);
    posPrevGyr = globalPosGyr(idx,:);
    
    % 4. Filtro de Kalman Lineal
    tic;  % Inicio medición de tiempo
    
    % Extrae velocidades angulares del giroscopio
    p = gyroData(1); q = gyroData(2); r = gyroData(3);
    
    % Matriz de transición de estado (modelo de cuaterniones)
    A = eye(4) + dt*1/2*[ 0  -p  -q  -r;
                         p   0   r  -q;
                         q  -r   0   p;
                         r   q  -p   0];
    
    % Observación: cuaternión derivado de medidas del acelerómetro y giroscopio
    z = eul2quat([phi_a(idx) theta_a(idx) -psi2], 'ZYX')';
    
    % Aplica el filtro de Kalman lineal
    [phi, theta, psi] = EulerKalman(A, z, phi_a(idx), theta_a(idx), 0);
    
    % Convierte a cuaternión y luego a ángulos de Euler para almacenar
    quatKF(idx) = quaternion([phi, theta, psi], 'euler', 'XYZ', 'frame');
    eulerKF(idx, :) = euler(quatKF(idx), 'ZYX', 'frame');
    
    % Transforma aceleración al marco global y calcula velocidad/posición por integración
    [globalAccKF(idx,:), globalVelKF(idx,:), globalPosKF(idx,:)] = ...
        transformAccelToGlobal(eulerKF(idx,:), accelData, dt, velPrevKF, posPrevKF);
    
    timeKF(idx) = toc;  % Fin medición de tiempo
    
    % Actualiza velocidad y posición previas para la siguiente iteración
    velPrevKF = globalVelKF(idx,:);
    posPrevKF = globalPosKF(idx,:);
    
    % 5. Filtro UDU
    tic;  % Inicio medición de tiempo
    
    % Aplica el filtro UDU para estimar orientación
    [phi, theta, psi] = EulerUDU([phi_a(idx) theta_a(idx)]', gyroData, dt, phi_a(idx), theta_a(idx), 0);
    
    % Convierte a cuaternión y luego a ángulos de Euler para almacenar
    quatUDU(idx) = quaternion([-phi, -theta, -psi], 'euler', 'XYZ', 'frame');
    eulerUDU(idx, :) = euler(quatUDU(idx), 'ZYX', 'frame');
    
    % Transforma aceleración al marco global y calcula velocidad/posición por integración
    [globalAccUDU(idx,:), globalVelUDU(idx,:), globalPosUDU(idx,:)] = ...
        transformAccelToGlobal(eulerUDU(idx,:), accelData, dt, velPrevUDU, posPrevUDU);
    
    timeUDU(idx) = toc;  % Fin medición de tiempo
    
    % Actualiza velocidad y posición previas para la siguiente iteración
    velPrevUDU = globalVelUDU(idx,:);
    posPrevUDU = globalPosUDU(idx,:);
    
    % 6. Filtro Takasu
    tic;  % Inicio medición de tiempo
    
    % Aplica el filtro Takasu para estimar orientación
    [phi, theta, psi] = EulerTakasu(-accelData, gyroData, dt, phi_a(idx), theta_a(idx), 0);
    eulerTakasu2(idx, :) = [phi, theta, psi];  % Guarda los ángulos directamente
    
    % Convierte a cuaternión y luego a ángulos de Euler para almacenar
    quatTakasu(idx) = quaternion([-phi, -theta, -psi], 'euler', 'XYZ', 'frame');
    eulerTakasu(idx, :) = euler(quatTakasu(idx), 'ZYX', 'frame');
    
    % Transforma aceleración al marco global y calcula velocidad/posición por integración
    [globalAccTakasu(idx,:), globalVelTakasu(idx,:), globalPosTakasu(idx,:)] = ...
        transformAccelToGlobal(eulerTakasu(idx,:), accelData, dt, velPrevTakasu, posPrevTakasu);
    
    timeTakasu(idx) = toc;  % Fin medición de tiempo
    
    % Actualiza velocidad y posición previas para la siguiente iteración
    velPrevTakasu = globalVelTakasu(idx,:);
    posPrevTakasu = globalPosTakasu(idx,:);
    
    % 7. Filtro Carlson
    tic;  % Inicio medición de tiempo
    
    % Aplica el filtro Carlson para estimar orientación
    [phi, theta, psi] = EulerCarlson([phi_a(idx) theta_a(idx)]', gyroData, dt, phi_a(idx), theta_a(idx), 0);
    
    % Convierte a cuaternión y luego a ángulos de Euler para almacenar
    quatCarlson(idx) = quaternion([-phi, -theta, -psi], 'euler', 'XYZ', 'frame');
    eulerCarlson(idx, :) = euler(quatCarlson(idx), 'ZYX', 'frame');
    
    % Transforma aceleración al marco global y calcula velocidad/posición por integración
    [globalAccCarlson(idx,:), globalVelCarlson(idx,:), globalPosCarlson(idx,:)] = ...
        transformAccelToGlobal(eulerCarlson(idx,:), accelData, dt, velPrevCarlson, posPrevCarlson);
    
    timeCarlson(idx) = toc;  % Fin medición de tiempo
    
    % Actualiza velocidad y posición previas para la siguiente iteración
    velPrevCarlson = globalVelCarlson(idx,:);
    posPrevCarlson = globalPosCarlson(idx,:);

    % 8. Filtro Takasu Mejorado
    tic;  % Inicio medición de tiempo
    
    % Aplica el filtro Takasu Mejorado para estimar orientación
    [phi, theta, psi] = EulerTakasu_mejorado([phi_a(idx) theta_a(idx)]', gyroData, dt, phi_a(idx), theta_a(idx), 0);
    
    % Convierte a cuaternión y luego a ángulos de Euler para almacenar
    quatTakasu_mejorado(idx) = quaternion([-phi, -theta, -psi], 'euler', 'XYZ', 'frame');
    eulerTakasu_mejorado(idx, :) = euler(quatTakasu_mejorado(idx), 'ZYX', 'frame');
    
    % Transforma aceleración al marco global y calcula velocidad/posición por integración
    [globalAccTakasu_mejorado(idx,:), globalVelTakasu_mejorado(idx,:), globalPosTakasu_mejorado(idx,:)] = ...
        transformAccelToGlobal(eulerTakasu_mejorado(idx,:), accelData, dt, velPrevTakasu_mejorado, posPrevTakasu_mejorado);
    
    timeTakasu_mejorado(idx) = toc;  % Fin medición de tiempo
    
    % Actualiza velocidad y posición previas para la siguiente iteración
    velPrevTakasu_mejorado = globalVelTakasu_mejorado(idx,:);
    posPrevTakasu_mejorado = globalPosTakasu_mejorado(idx,:);
end

%% 7. Análisis de tiempos de ejecución
% Calcula y muestra los tiempos promedio de ejecución para cada filtro
% Esto permite comparar la eficiencia computacional de cada implementación

avgTimeRef = mean(timeRef);                  % Tiempo promedio filtro Matlab
avgTimeEKF = mean(timeEKF);                  % Tiempo promedio EKF
avgTimeGyr = mean(timeGyr);                  % Tiempo promedio integración giroscopio
avgTimeKF = mean(timeKF);                    % Tiempo promedio Kalman Lineal
avgTimeUDU = mean(timeUDU);                  % Tiempo promedio UDU
avgTimeTakasu = mean(timeTakasu);            % Tiempo promedio Takasu
avgTimeCarlson = mean(timeCarlson);          % Tiempo promedio Carlson
avgTimeTakasu_mejorado = mean(timeTakasu_mejorado); % Tiempo promedio Takasu Mejorado

% Mostrar resultados en consola
fprintf('Tiempos de ejecución promedio:\n');
fprintf('Filtro Matlab: %.6f segundos\n', avgTimeRef);
fprintf('Filtro EKF: %.6f segundos\n', avgTimeEKF);
fprintf('Integración Giroscopio: %.6f segundos\n', avgTimeGyr);
fprintf('Filtro Kalman Lineal: %.6f segundos\n', avgTimeKF);
fprintf('Filtro UDU: %.6f segundos\n', avgTimeUDU);
fprintf('Filtro Takasu: %.6f segundos\n', avgTimeTakasu);
fprintf('Filtro Carlson: %.6f segundos\n', avgTimeCarlson);
fprintf('Filtro Takasu Mejorado: %.6f segundos\n', avgTimeTakasu_mejorado);

%% 8. Guardar resultados para análisis posterior
% Guarda todos los datos procesados en un archivo .mat para facilitar
% análisis adicionales o visualizaciones sin necesidad de reprocesar

save('resultados_filtros.mat', ...
    'eulerEst', 'globalAccMatlab', 'globalVelMatlab', 'globalPosMatlab', ...
    'eulerEKF', 'globalAccEKF', 'globalVelEKF', 'globalPosEKF', ...
    'eulerGyr', 'globalAccGyr', 'globalVelGyr', 'globalPosGyr', ...
    'eulerKF', 'globalAccKF', 'globalVelKF', 'globalPosKF', ...
    'eulerUDU', 'globalAccUDU', 'globalVelUDU', 'globalPosUDU', ...
    'eulerTakasu', 'globalAccTakasu', 'globalVelTakasu', 'globalPosTakasu', ...
    'eulerCarlson', 'globalAccCarlson', 'globalVelCarlson', 'globalPosCarlson', ...
    'eulerTakasu_mejorado', 'globalAccTakasu_mejorado', 'globalVelTakasu_mejorado', 'globalPosTakasu_mejorado', ...
    'timeRef', 'timeEKF', 'timeGyr', 'timeKF', 'timeUDU', 'timeTakasu', 'timeCarlson', 'timeTakasu_mejorado');

%% 9. Visualización de resultados
% Genera gráficos comparativos de los ángulos de Euler estimados por cada filtro
% Esto permite una evaluación visual del rendimiento de cada algoritmo

% Nombres de los ángulos de Euler para etiquetado
eulerNames = {'Yaw', 'Pitch', 'Roll'};

% Crear una figura para los ángulos de Euler
figure('Name', 'Comparación de Ángulos de Euler entre Filtros');

% Graficar cada ángulo de Euler en un subplot separado
for i = 1:3
    subplot(3,1,i);
    hold on;
    
    % Graficar resultados de cada filtro
    plot(rad2deg(eulerEKF(:,i)), 'r-', 'DisplayName', 'EKF');
    plot(rad2deg(eulerGyr(:,i)), 'b-', 'DisplayName', 'Gyro');
    plot(rad2deg(eulerKF(:,i)), 'g-', 'DisplayName', 'KF');
    plot(rad2deg(eulerUDU(:,i)), 'm-', 'DisplayName', 'UDU');
    plot(rad2deg(eulerTakasu(:,i)), 'c-', 'DisplayName', 'Takasu', 'LineWidth', 2);
    plot(rad2deg(eulerTakasu_mejorado(:,i)), 'r-', 'DisplayName', 'Takasu_mejorado', 'LineWidth', 2);
    plot(rad2deg(eulerCarlson(:,i)), 'y-', 'DisplayName', 'Carlson');
    plot(rad2deg(eulerEst(:,i)), '--', 'Color', [0.5 0.5 0.5], 'DisplayName', 'Matlab Est');
    
    % Añadir etiquetas y leyenda
    title(['Ángulo ' eulerNames{i}]);
    xlabel('Muestra'); ylabel('Ángulo (grados)');
    legend('show', 'Location', 'best'); grid on;
end

%% 10. Análisis Adicional (opcional)
% Esta sección puede ampliarse con análisis adicionales como:
% - Gráficos de aceleración global estimada
% - Gráficos de velocidad y posición estimadas
% - Análisis espectral de los datos
% - Comparación de trayectorias estimadas
% - Cálculo de métricas cuantitativas (error, desviación estándar)

% Figura para comparar aceleraciones globales
figure('Name', 'Comparación de Aceleraciones Globales');
titles = {'X', 'Y', 'Z'};

for i = 1:3
    subplot(3,1,i);
    hold on;
    
    % Graficar aceleraciones estimadas por cada filtro
    plot(globalAccEKF(:,i), 'r-', 'DisplayName', 'EKF');
    plot(globalAccGyr(:,i), 'b-', 'DisplayName', 'Gyro');
    plot(globalAccKF(:,i), 'g-', 'DisplayName', 'KF');
    plot(globalAccUDU(:,i), 'm-', 'DisplayName', 'UDU');
    plot(globalAccTakasu(:,i), 'c-', 'DisplayName', 'Takasu');
    plot(globalAccTakasu_mejorado(:,i), 'r-', 'DisplayName', 'Takasu_mejorado');
    plot(globalAccCarlson(:,i), 'y-', 'DisplayName', 'Carlson');
    plot(globalAccMatlab(:,i), '--', 'Color', [0.5 0.5 0.5], 'DisplayName', 'Matlab Est');
    
    % Añadir etiquetas y leyenda
    title(['Aceleración Global ' titles{i}]);
    xlabel('Muestra'); ylabel('Aceleración (m/s²)');
    legend('show', 'Location', 'best'); grid on;
end

%% 11. Interpretación y Conclusiones
% Esta sección puede ser completada con una interpretación de los resultados
% y conclusiones sobre el rendimiento de los diferentes filtros con datos reales.
%
% Algunos aspectos a considerar:
% - Estabilidad de las estimaciones de orientación con datos reales
% - Comportamiento ante ruido y perturbaciones reales
% - Comparación con los resultados de la simulación (si aplica)
% - Rendimiento computacional en aplicaciones prácticas
%
% Los resultados pueden variar significativamente con datos reales en comparación
% con simulaciones debido a:
% - Ruido de sensores reales no modelado correctamente en simulaciones
% - No-linealidades y comportamientos no ideales de los sensores
% - Errores de calibración y alineación mecánica
% - Perturbaciones externas (vibraciones, campos magnéticos, etc.)
%
% El filtro más adecuado dependerá de la aplicación específica y las
% características del hardware utilizado.

% Fin del script