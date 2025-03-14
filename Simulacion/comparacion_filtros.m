%% Análisis Comparativo de Filtros de Fusión Sensorial para IMU
% Este script evalúa y compara el desempeño de diferentes algoritmos de fusión 
% sensorial para estimar la orientación, velocidad y posición a partir de datos
% de una IMU (Unidad de Medición Inercial) simulada.
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
% - Filtro Solo Acelerómetro
%
% El script: 
%   1. Configura los parámetros de simulación
%   2. Genera una trayectoria de prueba
%   3. Configura una IMU simulada
%   4. Ejecuta cada uno de los filtros con datos simulados
%   5. Calcula métricas de error para cada filtro
%   6. Visualiza los resultados mediante gráficos comparativos

%% 1. Inicialización y configuración de la simulación
% Esta sección configura las rutas de directorios, frecuencias de muestreo 
% y parámetros iniciales necesarios para la simulación.

% Agregar directorios necesarios al path para acceder a las funciones de los filtros
addpath('GyroscopeIntegration');    % Funciones para integración del giroscopio
addpath('AccelerometerMagnetometer'); % Funciones para procesamiento de acelerómetro/magnetómetro
addpath('EulerKF');                 % Filtro de Kalman para ángulos de Euler
addpath('EulerEKF');                % Filtro de Kalman Extendido para ángulos de Euler
addpath('EulerUKF');                % Filtro de Kalman Unscented para ángulos de Euler
addpath('Functions');               % Funciones auxiliares generales

% Configuración de frecuencias de muestreo
imuFs = 400;                         % Frecuencia de muestreo de la IMU (Hz)
fs = 400;                            % Frecuencia general de la simulación (Hz)
localOrigin = [42.2825 -71.343 53.0352]; % Coordenadas del origen local [latitud, longitud, altitud]


%% 2. Inicialización del filtro de referencia de MATLAB
% Configura el filtro de navegación inercial no holonómico que servirá como referencia

gndFusion = insfilterNonholonomic('ReferenceFrame', 'ENU', ...
    'IMUSampleRate', imuFs, ...
    'ReferenceLocation', localOrigin, ...
    'DecimationFactor', 2);

%% 3. Generación de trayectoria de prueba
% Crea una trayectoria circular con orientación variable para probar los filtros
% Esta trayectoria sintética permite comparar las estimaciones con valores conocidos

% Parámetros de la trayectoria
r = 8.42;                          % Radio de la trayectoria circular (m)
speed = 2.50;                      % Velocidad lineal (m/s)
center = [0, 0];                   % Centro de la trayectoria (m)
initialYaw = 90;                   % Orientación inicial (grados)
numRevs = 2;                       % Número de revoluciones completas

% Calcula el tiempo para una revolución completa
revTime = 2*pi*r / speed;

% Genera puntos para la trayectoria
theta = (0:pi/2:2*pi*numRevs).';   % Ángulos para las posiciones
t = linspace(0, revTime*numRevs, numel(theta)).'; % Vector de tiempo

% Calcula coordenadas X, Y, Z para cada punto
x = r .* sin(theta) + center(1);
y = r .* sin(theta) + center(2);
z = zeros(size(x));                % Altura constante (suelo plano)
position = [x, y, z];              % Matriz de posiciones 3D

% Define la orientación en cada punto (cuaterniones)
pitch = theta;                     % El ángulo de inclinación varía con theta
pitch = mod(pitch, 2*pi);          % Normaliza el rango a [0, 2π]
roll = zeros(size(pitch));         % Sin rotación en el eje roll
yaw = zeros(size(pitch));          % Sin rotación en el eje yaw
orientation = quaternion([yaw, pitch, roll], 'euler', 'ZYX', 'frame'); % Crea cuaterniones

% Configura el objeto de trayectoria de waypoints
groundTruth = waypointTrajectory('SampleRate', imuFs, ...
    'Waypoints', position, ...
    'TimeOfArrival', t, ...
    'Orientation', orientation);

%% 4. Configuración de la IMU simulada
% Configura un sensor IMU simulado con características realistas

imu = imuSensor('accel-gyro', 'ReferenceFrame', 'ENU', 'SampleRate', imuFs);

% Configuración del acelerómetro
imu.Accelerometer.MeasurementRange = 19.6133;     % Rango de medición (m/s²)
imu.Accelerometer.Resolution = 0.0023928;         % Resolución (m/s²)
imu.Accelerometer.NoiseDensity = 0.0012356;       % Densidad de ruido (m/s²/√Hz)

% Configuración del giroscopio
imu.Gyroscope.MeasurementRange = deg2rad(250);    % Rango de medición (rad/s)
imu.Gyroscope.Resolution = deg2rad(0.0625);       % Resolución (rad/s)
imu.Gyroscope.NoiseDensity = deg2rad(0.025);      % Densidad de ruido (rad/s/√Hz)

%% 5. Configuración de estados iniciales
% Establece los estados iniciales para el filtro de referencia y la simulación

% Obtiene posición, actitud y velocidad iniciales desde la trayectoria
[initialPos, initialAtt, initialVel] = groundTruth();
reset(groundTruth); % Reinicia la trayectoria para comenzar la simulación

% Configuración del filtro de referencia con los valores iniciales
gndFusion.State(1:4) = compact(initialAtt).';          % Orientación (cuaternión)
gndFusion.State(5:7) = imu.Gyroscope.ConstantBias;     % Bias del giroscopio
gndFusion.State(8:10) = initialPos.';                  % Posición
gndFusion.State(11:13) = initialVel.';                 % Velocidad
gndFusion.State(14:16) = imu.Accelerometer.ConstantBias; % Bias del acelerómetro

% Parámetros de ruido para el filtro de referencia
gndFusion.ZeroVelocityConstraintNoise = 1e-2;   % Ruido en restricción de velocidad cero
gndFusion.GyroscopeNoise = 4e-6;                % Ruido del giroscopio
gndFusion.GyroscopeBiasNoise = 4e-14;           % Ruido del bias del giroscopio
gndFusion.AccelerometerNoise = 4.8e-2;          % Ruido del acelerómetro
gndFusion.AccelerometerBiasNoise = 4e-14;       % Ruido del bias del acelerómetro
gndFusion.StateCovariance = 1e-9*eye(16);       % Covarianza de estado inicial

%% 6. Inicialización de variables para almacenar resultados
% Esta sección define todas las variables que almacenarán los resultados de cada filtro

totalSimTime = 30;                               % Tiempo total de simulación (s)
numsamples = floor(min(t(end), totalSimTime) * imuFs); % Número de muestras

% Posición y orientación verdadera (ground truth)
truePosition = zeros(numsamples, 3);            % Posición real [x,y,z]
trueOrientation = quaternion.zeros(numsamples, 1); % Orientación real (cuaterniones)
eulerTrue = zeros(numsamples, 3);

% Variables para cada filtro (almacenamiento de resultados)
% Filtro de Referencia de MATLAB
estPosition = zeros(numsamples, 3);             % Posición estimada
estOrientation = quaternion.zeros(numsamples, 1); % Orientación estimada
eulerEst = zeros(numsamples, 3);                % Ángulos de Euler estimados
globalAccMatlab = zeros(numsamples, 3);         % Aceleración global estimada
globalVelMatlab = zeros(numsamples, 3);         % Velocidad global estimada
globalPosMatlab = zeros(numsamples, 3);         % Posición global estimada
timeRef = zeros(numsamples, 1);                 % Tiempo de ejecución

% Filtro EKF (Extended Kalman Filter)
eulerEKF = zeros(numsamples, 3);                % Ángulos de Euler estimados
globalAccEKF = zeros(numsamples, 3);            % Aceleración global estimada
globalVelEKF = zeros(numsamples, 3);            % Velocidad global estimada 
globalPosEKF = zeros(numsamples, 3);            % Posición global estimada
timeEKF = zeros(numsamples, 1);                 % Tiempo de ejecución

% Integración de Giroscopio
eulerGyr = zeros(numsamples, 3);                % Ángulos de Euler estimados
globalAccGyr = zeros(numsamples, 3);            % Aceleración global estimada
globalVelGyr = zeros(numsamples, 3);            % Velocidad global estimada
globalPosGyr = zeros(numsamples, 3);            % Posición global estimada
timeGyr = zeros(numsamples, 1);                 % Tiempo de ejecución

% Filtro de Kalman Lineal
eulerKF = zeros(numsamples, 3);                 % Ángulos de Euler estimados
globalAccKF = zeros(numsamples, 3);             % Aceleración global estimada
globalVelKF = zeros(numsamples, 3);             % Velocidad global estimada
globalPosKF = zeros(numsamples, 3);             % Posición global estimada
timeKF = zeros(numsamples, 1);                  % Tiempo de ejecución

% Ground Truth
globalAccTrue = zeros(numsamples, 3);           % Aceleración global real
globalVelTrue = zeros(numsamples, 3);           % Velocidad global real
globalPosTrue = zeros(numsamples, 3);           % Posición global real

% Filtro UDU
eulerUDU = zeros(numsamples, 3);                % Ángulos de Euler estimados
globalAccUDU = zeros(numsamples, 3);            % Aceleración global estimada
globalVelUDU = zeros(numsamples, 3);            % Velocidad global estimada
globalPosUDU = zeros(numsamples, 3);            % Posición global estimada
timeUDU = zeros(numsamples, 1);                 % Tiempo de ejecución

% Filtro Takasu
eulerTakasu = zeros(numsamples, 3);             % Ángulos de Euler estimados
globalAccTakasu = zeros(numsamples, 3);         % Aceleración global estimada
globalVelTakasu = zeros(numsamples, 3);         % Velocidad global estimada
globalPosTakasu = zeros(numsamples, 3);         % Posición global estimada
timeTakasu = zeros(numsamples, 1);              % Tiempo de ejecución

% Filtro Carlson
eulerCarlson = zeros(numsamples, 3);            % Ángulos de Euler estimados
globalAccCarlson = zeros(numsamples, 3);        % Aceleración global estimada
globalVelCarlson = zeros(numsamples, 3);        % Velocidad global estimada
globalPosCarlson = zeros(numsamples, 3);        % Posición global estimada
timeCarlson = zeros(numsamples, 1);             % Tiempo de ejecución

% Filtro Takasu Mejorado
eulerTakasu_mejorado = zeros(numsamples, 3);    % Ángulos de Euler estimados
globalAccTakasu_mejorado = zeros(numsamples, 3); % Aceleración global estimada
globalVelTakasu_mejorado = zeros(numsamples, 3); % Velocidad global estimada
globalPosTakasu_mejorado = zeros(numsamples, 3); % Posición global estimada
timeTakasu_mejorado = zeros(numsamples, 1);     % Tiempo de ejecución

% Solo Acelerómetro
eulerAccel_only = zeros(numsamples, 3);         % Ángulos de Euler estimados
globalAccAccel_only = zeros(numsamples, 3);     % Aceleración global estimada
globalVelAccel_only = zeros(numsamples, 3);     % Velocidad global estimada
globalPosAccel_only = zeros(numsamples, 3);     % Posición global estimada
timeAccel_only = zeros(numsamples, 1);          % Tiempo de ejecución

% Variables para velocidades y posiciones previas (necesarias para integración)
velPrevUDU = [0, 0, 0];                         % Velocidad previa UDU
posPrevUDU = [0, 0, 0];                         % Posición previa UDU
velPrevTakasu = [0, 0, 0];                      % Velocidad previa Takasu
posPrevTakasu = [0, 0, 0];                      % Posición previa Takasu
velPrevCarlson = [0, 0, 0];                     % Velocidad previa Carlson
posPrevCarlson = [0, 0, 0];                     % Posición previa Carlson
velPrevTakasu_mejorado = [0, 0, 0];             % Velocidad previa Takasu Mejorado
posPrevTakasu_mejorado = [0, 0, 0];             % Posición previa Takasu Mejorado
velPrevAccel_only = [0, 0, 0];                  % Velocidad previa Solo Acelerómetro
posPrevAccel_only = [0, 0, 0];                  % Posición previa Solo Acelerómetro

% Inicialización de vectores para almacenar cuaterniones
quatEKF = quaternion.zeros(numsamples, 1);
quatGyr = quaternion.zeros(numsamples, 1);
quatKF = quaternion.zeros(numsamples, 1);
quatUDU = quaternion.zeros(numsamples, 1);
quatTakasu = quaternion.zeros(numsamples, 1);
quatCarlson = quaternion.zeros(numsamples, 1);
quatTakasu_mejorado = quaternion.zeros(numsamples, 1);
quatAccel_only = quaternion.zeros(numsamples, 1);

%% 7. Loop principal de simulación
% Esta sección ejecuta la simulación, aplica cada filtro en cada paso de tiempo
% y almacena los resultados para su posterior análisis

% Limpia instancias previas de los objetos de los filtros
clear EulerCarlson;
clear EulerTakasu;
clear EulerUDU;
clear EulerKalman;
clear EulerGyro;
clear EulerEKF;
clear EulerTakasu_mejorado;

for idx = 1:numsamples
    if ~isDone(groundTruth)
        % Obtener datos de ground truth desde la trayectoria simulada
        [truePosition(idx,:), trueOrientation(idx,:), trueVel, trueAcc, trueAngVel] = groundTruth();
        
        % Simular lecturas de la IMU basadas en los valores reales
        [accelData, gyroData] = imu(trueAcc, trueAngVel, trueOrientation(idx,:));
        
        % Intervalo de tiempo para integración
        dt = 1/fs;

        % 1. Filtro de referencia Matlab
        tic; % Inicio medición de tiempo
        % Actualiza el filtro con las mediciones actuales
        predict(gndFusion, accelData, gyroData);
        
        % Obtiene la posición y orientación estimadas
        [estPosition(idx,:), estOrientation(idx,:)] = pose(gndFusion);
        
        % Convierte orientación a ángulos de Euler
        eulerEst(idx, :) = euler(estOrientation(idx,:), 'ZYX', 'frame');
        
        % Transforma aceleración a marco global y calcula velocidad/posición por integración
        [globalAccMatlab(idx,:), globalVelMatlab(idx,:), globalPosMatlab(idx,:)] = ...
            transformAccelToGlobal(eulerEst(idx,:), accelData, dt, ...
            globalVelMatlab(max(1,idx-1),:), globalPosMatlab(max(1,idx-1),:));
        
        timeRef(idx) = toc; % Fin medición de tiempo

        % 2. Filtro EKF (Extended Kalman Filter)
        tic; % Inicio medición de tiempo
        
        % Obtiene orientación inicial de la trayectoria
        first_orientation = euler(trueOrientation(1,1), 'ZYX', 'frame');
        
        % Estima ángulos de Euler (roll, pitch) a partir del acelerómetro
        [phi_a, theta_a] = EulerAccel(-accelData(1), -accelData(2), -accelData(3), 0, 0, 0);
        
        % Aplica el filtro EKF para estimar orientación completa
        [phi, theta, psi] = EulerEKF([phi_a theta_a]', gyroData, dt, -phi_a, -theta_a, -first_orientation(1));
        
        % Convierte a cuaternión y luego a ángulos de Euler para almacenar
        quatEKF(idx) = quaternion([-phi, -theta, -psi], 'euler', 'XYZ', 'frame');
        eulerEKF(idx, :) = euler(quatEKF(idx), 'ZYX', 'frame');
        
        % Transforma aceleración a marco global y calcula velocidad/posición por integración
        [globalAccEKF(idx,:), globalVelEKF(idx,:), globalPosEKF(idx,:)] = ...
            transformAccelToGlobal(eulerEKF(idx,:), accelData, dt, ...
            globalVelEKF(max(1,idx-1),:), globalPosEKF(max(1,idx-1),:));
        
        timeEKF(idx) = toc; % Fin medición de tiempo
        
        % 3. Integración del Giroscopio
        tic; % Inicio medición de tiempo
        
        % Integra las velocidades angulares para estimar orientación
        [phi2, theta2, psi2] = EulerGyro(-gyroData(1), -gyroData(2), -gyroData(3), ...
            dt, -phi_a, -theta_a, -first_orientation(1));
        
        % Convierte a cuaternión y luego a ángulos de Euler para almacenar
        quatGyr(idx) = quaternion([-phi2, -theta2, -psi2], 'euler', 'XYZ', 'frame');
        eulerGyr(idx, :) = euler(quatGyr(idx), 'ZYX', 'frame');
        
        % Transforma aceleración a marco global y calcula velocidad/posición por integración
        [globalAccGyr(idx,:), globalVelGyr(idx,:), globalPosGyr(idx,:)] = ...
            transformAccelToGlobal(eulerGyr(idx,:), accelData, dt, ...
            globalVelGyr(max(1,idx-1),:), globalPosGyr(max(1,idx-1),:));
        
        timeGyr(idx) = toc; % Fin medición de tiempo
        
        % 4. Filtro de Kalman Lineal
        tic; % Inicio medición de tiempo
        
        % Extrae velocidades angulares del giroscopio
        p = gyroData(1); q = gyroData(2); r = gyroData(3);
        
        % Matriz de transición de estado (modelo de cuaterniones)
        A = eye(4) + dt*1/2*[ 0  -p  -q  -r;
                             p   0   r  -q;
                             q  -r   0   p;
                             r   q  -p   0];
        
        % Observación: cuaternión derivado de medidas del acelerómetro y giroscopio
        z = eul2quat([-phi_a -theta_a psi2], 'ZYX')';
        
        % Aplica el filtro de Kalman lineal
        [phi, theta, psi] = EulerKalman(A, z, phi_a, theta_a, -first_orientation(1));
        
        % Convierte a cuaternión y luego a ángulos de Euler para almacenar
        quatKF(idx) = quaternion([-phi, -theta, -psi], 'euler', 'XYZ', 'frame');
        eulerKF(idx, :) = euler(quatKF(idx), 'ZYX', 'frame');
        
        % Transforma aceleración a marco global y calcula velocidad/posición por integración
        [globalAccKF(idx,:), globalVelKF(idx,:), globalPosKF(idx,:)] = ...
            transformAccelToGlobal(eulerKF(idx,:), accelData, dt, ...
            globalVelKF(max(1,idx-1),:), globalPosKF(max(1,idx-1),:));
        
        timeKF(idx) = toc; % Fin medición de tiempo
        
        % 5. Filtro UDU
        tic; % Inicio medición de tiempo
        
        % Aplica el filtro UDU para estimar orientación
        [phi, theta, psi] = EulerUDU([phi_a theta_a]', gyroData, dt, phi_a, theta_a, first_orientation(1));
        
        % Convierte a cuaternión y luego a ángulos de Euler para almacenar
        quatUDU(idx) = quaternion([-phi, -theta, -psi], 'euler', 'XYZ', 'frame');
        eulerUDU(idx, :) = euler(quatUDU(idx), 'ZYX', 'frame');
        
        % Transforma aceleración a marco global y calcula velocidad/posición por integración
        [globalAccUDU(idx,:), globalVelUDU(idx,:), globalPosUDU(idx,:)] = ...
            transformAccelToGlobal(eulerUDU(idx,:), accelData, dt, velPrevUDU, posPrevUDU);
        
        % Actualiza velocidad y posición previas para la siguiente iteración
        velPrevUDU = globalVelUDU(idx,:);
        posPrevUDU = globalPosUDU(idx,:);
        
        timeUDU(idx) = toc; % Fin medición de tiempo
        
        % 6. Filtro Takasu
        tic; % Inicio medición de tiempo
        
        % Aplica el filtro Takasu para estimar orientación
        [phi, theta, psi] = EulerTakasu(-accelData, gyroData, dt, phi_a, theta_a, first_orientation(1));
        
        % Convierte a cuaternión y luego a ángulos de Euler para almacenar
        quatTakasu(idx) = quaternion([-phi, -theta, -psi], 'euler', 'XYZ', 'frame');
        eulerTakasu(idx, :) = euler(quatTakasu(idx), 'ZYX', 'frame');
        
        % Transforma aceleración a marco global y calcula velocidad/posición por integración
        [globalAccTakasu(idx,:), globalVelTakasu(idx,:), globalPosTakasu(idx,:)] = ...
            transformAccelToGlobal(eulerTakasu(idx,:), accelData, dt, velPrevTakasu, posPrevTakasu);
        
        % Actualiza velocidad y posición previas para la siguiente iteración
        velPrevTakasu = globalVelTakasu(idx,:);
        posPrevTakasu = globalPosTakasu(idx,:);
        
        timeTakasu(idx) = toc; % Fin medición de tiempo
        
        % 7. Filtro Carlson
        tic; % Inicio medición de tiempo
        
        % Aplica el filtro Carlson para estimar orientación
        [phi, theta, psi] = EulerCarlson([phi_a theta_a]', gyroData, dt, phi_a, theta_a, first_orientation(1));
        
        % Convierte a cuaternión y luego a ángulos de Euler para almacenar
        quatCarlson(idx) = quaternion([-phi, -theta, -psi], 'euler', 'XYZ', 'frame');
        eulerCarlson(idx, :) = euler(quatCarlson(idx), 'ZYX', 'frame');
        
        % Transforma aceleración a marco global y calcula velocidad/posición por integración
        [globalAccCarlson(idx,:), globalVelCarlson(idx,:), globalPosCarlson(idx,:)] = ...
            transformAccelToGlobal(eulerCarlson(idx,:), accelData, dt, velPrevCarlson, posPrevCarlson);
        
        % Actualiza velocidad y posición previas para la siguiente iteración
        velPrevCarlson = globalVelCarlson(idx,:);
        posPrevCarlson = globalPosCarlson(idx,:);
        
        timeCarlson(idx) = toc; % Fin medición de tiempo

        % 8. Filtro Takasu Mejorado
        tic; % Inicio medición de tiempo
        
        % Aplica el filtro Takasu Mejorado para estimar orientación
        [phi, theta, psi] = EulerTakasu_mejorado([phi_a theta_a]', gyroData, dt, phi_a, theta_a, first_orientation(1));
        
        % Convierte a cuaternión y luego a ángulos de Euler para almacenar
        quatTakasu_mejorado(idx) = quaternion([-phi, -theta, -psi], 'euler', 'XYZ', 'frame');
        eulerTakasu_mejorado(idx, :) = euler(quatTakasu_mejorado(idx), 'ZYX', 'frame');
        
        % Transforma aceleración a marco global y calcula velocidad/posición por integración
        [globalAccTakasu_mejorado(idx,:), globalVelTakasu_mejorado(idx,:), globalPosTakasu_mejorado(idx,:)] = ...
            transformAccelToGlobal(eulerTakasu_mejorado(idx,:), accelData, dt, ...
            velPrevTakasu_mejorado, posPrevTakasu_mejorado);
        
        % Actualiza velocidad y posición previas para la siguiente iteración
        velPrevTakasu_mejorado = globalVelTakasu_mejorado(idx,:);
        posPrevTakasu_mejorado = globalPosTakasu_mejorado(idx,:);
        
        timeTakasu_mejorado(idx) = toc; % Fin medición de tiempo

        % 9. Solo Acelerómetro (baseline para comparación)
        tic; % Inicio medición de tiempo
        
        % Estima solo roll y pitch del acelerómetro (yaw no es observable)
        [phi_accel, theta_accel] = EulerAccel(-accelData(1), -accelData(2), -accelData(3), 0, 0, 0);
        
        % El yaw se mantiene en 0 para la estimación por acelerómetro
        psi_accel = 0;
        
        % Convierte a cuaternión y luego a ángulos de Euler para almacenar
        quatAccel_only(idx) = quaternion([-phi_accel, -theta_accel, -psi_accel], 'euler', 'XYZ', 'frame');
        eulerAccel_only(idx, :) = euler(quatAccel_only(idx), 'ZYX', 'frame');
        
        % Transforma aceleración a marco global y calcula velocidad/posición por integración
        [globalAccAccel_only(idx,:), globalVelAccel_only(idx,:), globalPosAccel_only(idx,:)] = ...
            transformAccelToGlobal(eulerAccel_only(idx,:), accelData, dt, ...
            velPrevAccel_only, posPrevAccel_only);
        
        % Actualiza velocidad y posición previas para la siguiente iteración
        velPrevAccel_only = globalVelAccel_only(idx,:);
        posPrevAccel_only = globalPosAccel_only(idx,:);
        
        timeAccel_only(idx) = toc; % Fin medición de tiempo

        % Registro de datos de Ground Truth para comparación
        eulerTrue(idx, :) = euler(trueOrientation(idx,:), 'ZYX', 'frame');
        [globalAccTrue(idx,:), globalVelTrue(idx,:), globalPosTrue(idx,:)] = ...
            transformAccelToGlobal(eulerTrue(idx, :), accelData, dt, ...
            globalVelTrue(max(1,idx-1),:), globalPosTrue(max(1,idx-1),:));
    end
end

%% 8. Análisis de Resultados - Gráficos de Estimaciones y Errores
% Esta sección genera visualizaciones para comparar el rendimiento de los diferentes filtros
% Se crean gráficos para aceleración, velocidad, posición y orientación

titles = {'X', 'Y', 'Z'};                 % Títulos para los ejes
eulerNames = {'Yaw', 'Pitch', 'Roll'};    % Nombres de los ángulos de Euler

% Función para calcular el error angular mínimo (en radianes)
% Considera la periodicidad de los ángulos (ej: 359° y 1° tienen diferencia de 2°, no 358°)
calc_min_error = @(est, true) min(abs(est - true), 2*pi - abs(est - true));

% 1. Gráficos de Aceleración Global
figure('Name', 'Aceleración Global - Todos los Filtros');
for i = 1:3
    subplot(3, 1, i);
    hold on;
    plot(globalAccTrue(:,i), 'k--', 'DisplayName', 'Ground Truth', 'LineWidth', 2);
    plot(globalAccEKF(:,i), 'r-', 'DisplayName', 'EKF');
    plot(globalAccGyr(:,i), 'b-', 'DisplayName', 'Gyro');
    plot(globalAccKF(:,i), 'g-', 'DisplayName', 'KF');
    plot(globalAccUDU(:,i), 'm-', 'DisplayName', 'UDU');
    plot(globalAccTakasu(:,i), 'c-', 'DisplayName', 'Takasu');
    plot(globalAccCarlson(:,i), 'y-', 'DisplayName', 'Carlson');
    plot(globalAccTakasu_mejorado(:,i), 'Color', [0.8 0.4 0], 'DisplayName', 'Takasu Mejorado');
    plot(globalAccAccel_only(:,i), 'Color', [0.5 0.5 0.8], 'DisplayName', 'Solo Acelerómetro');
    title(['Aceleración Global ' titles{i}]);
    xlabel('Muestra'); ylabel('Aceleración (m/s^2)');
    legend('show', 'Location', 'best'); grid on;
end

% 2. Gráficos de Errores de Aceleración Global
figure('Name', 'Errores de Aceleración Global - Todos los Filtros');
for i = 1:3
    subplot(3, 1, i);
    hold on;
    plot(abs(globalAccEKF(:,i) - globalAccTrue(:,i)), 'r-', 'DisplayName', 'Error EKF');
    plot(abs(globalAccGyr(:,i) - globalAccTrue(:,i)), 'b-', 'DisplayName', 'Error Gyro');
    plot(abs(globalAccKF(:,i) - globalAccTrue(:,i)), 'g-', 'DisplayName', 'Error KF');
    plot(abs(globalAccUDU(:,i) - globalAccTrue(:,i)), 'm-', 'DisplayName', 'Error UDU');
    plot(abs(globalAccTakasu(:,i) - globalAccTrue(:,i)), 'c-', 'DisplayName', 'Error Takasu');
    plot(abs(globalAccCarlson(:,i) - globalAccTrue(:,i)), 'y-', 'DisplayName', 'Error Carlson');
    plot(abs(globalAccTakasu_mejorado(:,i) - globalAccTrue(:,i)), 'Color', [0.8 0.4 0], 'DisplayName', 'Error Takasu Mejorado');
    plot(abs(globalAccAccel_only(:,i) - globalAccTrue(:,i)), 'Color', [0.5 0.5 0.8], 'DisplayName', 'Error Solo Acelerómetro');
    title(['Error de Aceleración Global ' titles{i}]);
    xlabel('Muestra'); ylabel('Error (m/s^2)');
    legend('show', 'Location', 'best'); grid on;
end

% 3. Gráficos de Velocidad Global
figure('Name', 'Velocidad Global - Todos los Filtros');
for i = 1:3
    subplot(3, 1, i);
    hold on;
    plot(globalVelTrue(:,i), 'k--', 'DisplayName', 'Ground Truth', 'LineWidth', 2);
    plot(globalVelEKF(:,i), 'r-', 'DisplayName', 'EKF');
    plot(globalVelGyr(:,i), 'b-', 'DisplayName', 'Gyro');
    plot(globalVelKF(:,i), 'g-', 'DisplayName', 'KF');
    plot(globalVelUDU(:,i), 'm-', 'DisplayName', 'UDU');
    plot(globalVelTakasu(:,i), 'c-', 'DisplayName', 'Takasu');
    plot(globalVelCarlson(:,i), 'y-', 'DisplayName', 'Carlson');
    plot(globalVelTakasu_mejorado(:,i), 'Color', [0.8 0.4 0], 'DisplayName', 'Takasu Mejorado');
    plot(globalVelAccel_only(:,i), 'Color', [0.5 0.5 0.8], 'DisplayName', 'Solo Acelerómetro');
    title(['Velocidad Global ' titles{i}]);
    xlabel('Muestra'); ylabel('Velocidad (m/s)');
    legend('show', 'Location', 'best'); grid on;
end

% 4. Gráficos de Errores de Velocidad Global
figure('Name', 'Errores de Velocidad Global - Todos los Filtros');
for i = 1:3
    subplot(3, 1, i);
    hold on;
    plot(abs(globalVelEKF(:,i) - globalVelTrue(:,i)), 'r-', 'DisplayName', 'Error EKF');
    plot(abs(globalVelGyr(:,i) - globalVelTrue(:,i)), 'b-', 'DisplayName', 'Error Gyro');
    plot(abs(globalVelKF(:,i) - globalVelTrue(:,i)), 'g-', 'DisplayName', 'Error KF');
    plot(abs(globalVelUDU(:,i) - globalVelTrue(:,i)), 'm-', 'DisplayName', 'Error UDU');
    plot(abs(globalVelTakasu(:,i) - globalVelTrue(:,i)), 'c-', 'DisplayName', 'Error Takasu');
    plot(abs(globalVelCarlson(:,i) - globalVelTrue(:,i)), 'y-', 'DisplayName', 'Error Carlson');
    plot(abs(globalVelTakasu_mejorado(:,i) - globalVelTrue(:,i)), 'Color', [0.8 0.4 0], 'DisplayName', 'Error Takasu Mejorado');
    plot(abs(globalVelAccel_only(:,i) - globalVelTrue(:,i)), 'Color', [0.5 0.5 0.8], 'DisplayName', 'Error Solo Acelerómetro');
    title(['Error de Velocidad Global ' titles{i}]);
    xlabel('Muestra'); ylabel('Error (m/s)');
    legend('show', 'Location', 'best'); grid on;
end

% 5. Gráficos de Posición Global
figure('Name', 'Posición Global - Todos los Filtros');
for i = 1:3
    subplot(3, 1, i);
    hold on;
    plot(globalPosTrue(:,i), 'k--', 'DisplayName', 'Ground Truth', 'LineWidth', 2);
    plot(globalPosEKF(:,i), 'r-', 'DisplayName', 'EKF');
    plot(globalPosGyr(:,i), 'b-', 'DisplayName', 'Gyro');
    plot(globalPosKF(:,i), 'g-', 'DisplayName', 'KF');
    plot(globalPosUDU(:,i), 'm-', 'DisplayName', 'UDU');
    plot(globalPosTakasu(:,i), 'c-', 'DisplayName', 'Takasu');
    plot(globalPosCarlson(:,i), 'y-', 'DisplayName', 'Carlson');
    plot(globalPosTakasu_mejorado(:,i), 'Color', [0.8 0.4 0], 'DisplayName', 'Takasu Mejorado');
    plot(globalPosAccel_only(:,i), 'Color', [0.5 0.5 0.8], 'DisplayName', 'Solo Acelerómetro');
    plot(estPosition(:,i), 'Color', [0.5 0.5 0.5], 'DisplayName', 'Matlab Est');
    title(['Posición Global ' titles{i}]);
    xlabel('Muestra'); ylabel('Posición (m)');
    legend('show', 'Location', 'best'); grid on;
end

% 6. Gráficos de Errores de Posición Global
figure('Name', 'Errores de Posición Global - Todos los Filtros');
for i = 1:3
    subplot(3, 1, i);
    hold on;
    plot(abs(globalPosEKF(:,i) - globalPosTrue(:,i)), 'r-', 'DisplayName', 'Error EKF');
    plot(abs(globalPosGyr(:,i) - globalPosTrue(:,i)), 'b-', 'DisplayName', 'Error Gyro');
    plot(abs(globalPosKF(:,i) - globalPosTrue(:,i)), 'g-', 'DisplayName', 'Error KF');
    plot(abs(globalPosUDU(:,i) - globalPosTrue(:,i)), 'm-', 'DisplayName', 'Error UDU');
    plot(abs(globalPosTakasu(:,i) - globalPosTrue(:,i)), 'c-', 'DisplayName', 'Error Takasu');
    plot(abs(globalPosCarlson(:,i) - globalPosTrue(:,i)), 'y-', 'DisplayName', 'Error Carlson');
    plot(abs(globalPosTakasu_mejorado(:,i) - globalPosTrue(:,i)), 'Color', [0.8 0.4 0], 'DisplayName', 'Error Takasu Mejorado');
    plot(abs(globalPosAccel_only(:,i) - globalPosTrue(:,i)), 'Color', [0.5 0.5 0.8], 'DisplayName', 'Error Solo Acelerómetro');
    plot(abs(estPosition(:,i) - globalPosTrue(:,i)), 'Color', [0.5 0.5 0.5], 'DisplayName', 'Error Matlab Est');
    title(['Error de Posición Global ' titles{i}]);
    xlabel('Muestra'); ylabel('Error (m)');
    legend('show', 'Location', 'best'); grid on;
end

% 7. Gráficos de Ángulos de Euler
figure('Name', 'Ángulos de Euler - Todos los Filtros');
for i = 1:3
    subplot(3, 1, i);
    hold on;
    plot(rad2deg(eulerTrue(:,i)), 'k--', 'DisplayName', 'Ground Truth', 'LineWidth', 2);
    plot(rad2deg(eulerEKF(:,i)), 'r-', 'DisplayName', 'EKF');
    plot(rad2deg(eulerGyr(:,i)), 'b-', 'DisplayName', 'Gyro');
    plot(rad2deg(eulerKF(:,i)), 'g-', 'DisplayName', 'KF');
    plot(rad2deg(eulerUDU(:,i)), 'm-', 'DisplayName', 'UDU');
    plot(rad2deg(eulerTakasu(:,i)), 'c-', 'DisplayName', 'Takasu');
    plot(rad2deg(eulerCarlson(:,i)), 'y-', 'DisplayName', 'Carlson');
    plot(rad2deg(eulerTakasu_mejorado(:,i)), 'Color', [0.8 0.4 0], 'DisplayName', 'Takasu Mejorado');
    plot(rad2deg(eulerAccel_only(:,i)), 'Color', [0.5 0.5 0.8], 'DisplayName', 'Solo Acelerómetro');
    title(['Ángulo ' eulerNames{i}]);
    xlabel('Muestra'); ylabel('Ángulo (°)');
    legend('show', 'Location', 'best'); grid on;
end

% 8. Gráficos de Errores de Ángulos de Euler
figure('Name', 'Errores Ajustados de Ángulos de Euler - Todos los Filtros');
for i = 1:3
    subplot(3, 1, i);
    hold on;
    % Cálculo del error angular mínimo en radianes, luego se convierte a grados
    plot(rad2deg(calc_min_error(eulerEKF(:,i), eulerTrue(:,i))), 'r-', 'DisplayName', 'Error EKF');
    plot(rad2deg(calc_min_error(eulerGyr(:,i), eulerTrue(:,i))), 'b-', 'DisplayName', 'Error Gyro');
    plot(rad2deg(calc_min_error(eulerKF(:,i), eulerTrue(:,i))), 'g-', 'DisplayName', 'Error KF');
    plot(rad2deg(calc_min_error(eulerUDU(:,i), eulerTrue(:,i))), 'm-', 'DisplayName', 'Error UDU');
    plot(rad2deg(calc_min_error(eulerTakasu(:,i), eulerTrue(:,i))), 'c-', 'DisplayName', 'Error Takasu');
    plot(rad2deg(calc_min_error(eulerCarlson(:,i), eulerTrue(:,i))), 'y-', 'DisplayName', 'Error Carlson');
    plot(rad2deg(calc_min_error(eulerTakasu_mejorado(:,i), eulerTrue(:,i))), 'Color', [0.8 0.4 0], 'DisplayName', 'Error Takasu Mejorado');
    plot(rad2deg(calc_min_error(eulerAccel_only(:,i), eulerTrue(:,i))), 'Color', [0.5 0.5 0.8], 'DisplayName', 'Error Solo Acelerómetro');
    title(['Error Ajustado de Ángulo ' eulerNames{i}]);
    xlabel('Muestra'); ylabel('Error (°)');
    legend('show', 'Location', 'best'); grid on;
end

%% 9. Cálculo de Errores RMS y Error Acumulado
% Esta sección calcula métricas cuantitativas para evaluar el rendimiento de cada filtro
% Incluye el Error RMS (Root Mean Square) y Error Acumulado para cada variable

% Funciones para cálculo de errores
calcRMS = @(x,y) sqrt(mean((x-y).^2));              % Error RMS entre dos señales
calcAccumError = @(x,y) sum(abs(x-y));              % Error acumulado (suma de errores absolutos)

% Función para calcular RMS promedio de los 3 ejes
calcAvgRMS = @(x,y) mean([calcRMS(x(:,1), y(:,1)), calcRMS(x(:,2), y(:,2)), calcRMS(x(:,3), y(:,3))]);

% Función para calcular error acumulado promedio de los 3 ejes
calcAvgAccumError = @(x,y) mean([calcAccumError(x(:,1), y(:,1)), calcAccumError(x(:,2), y(:,2)), calcAccumError(x(:,3), y(:,3))]);

% 1. Errores de Aceleración Promedio
accRMS_EKF = calcAvgRMS(globalAccTrue, globalAccEKF);
accRMS_Gyr = calcAvgRMS(globalAccTrue, globalAccGyr);
accRMS_KF = calcAvgRMS(globalAccTrue, globalAccKF);
accRMS_Matlab = calcAvgRMS(globalAccTrue, globalAccMatlab);
accRMS_UDU = calcAvgRMS(globalAccTrue, globalAccUDU);
accRMS_Takasu = calcAvgRMS(globalAccTrue, globalAccTakasu);
accRMS_Carlson = calcAvgRMS(globalAccTrue, globalAccCarlson);
accRMS_Takasu_mejorado = calcAvgRMS(globalAccTrue, globalAccTakasu_mejorado);
accRMS_Accel_only = calcAvgRMS(globalAccTrue, globalAccAccel_only);

% Error Acumulado de Aceleración
accAccum_EKF = calcAvgAccumError(globalAccTrue, globalAccEKF);
accAccum_Gyr = calcAvgAccumError(globalAccTrue, globalAccGyr);
accAccum_KF = calcAvgAccumError(globalAccTrue, globalAccKF);
accAccum_Matlab = calcAvgAccumError(globalAccTrue, globalAccMatlab);
accAccum_UDU = calcAvgAccumError(globalAccTrue, globalAccUDU);
accAccum_Takasu = calcAvgAccumError(globalAccTrue, globalAccTakasu);
accAccum_Carlson = calcAvgAccumError(globalAccTrue, globalAccCarlson);
accAccum_Takasu_mejorado = calcAvgAccumError(globalAccTrue, globalAccTakasu_mejorado);
accAccum_Accel_only = calcAvgAccumError(globalAccTrue, globalAccAccel_only);

% 2. Errores de Velocidad Promedio
velRMS_EKF = calcAvgRMS(globalVelTrue, globalVelEKF);
velRMS_Gyr = calcAvgRMS(globalVelTrue, globalVelGyr);
velRMS_KF = calcAvgRMS(globalVelTrue, globalVelKF);
velRMS_Matlab = calcAvgRMS(globalVelTrue, globalVelMatlab);
velRMS_UDU = calcAvgRMS(globalVelTrue, globalVelUDU);
velRMS_Takasu = calcAvgRMS(globalVelTrue, globalVelTakasu);
velRMS_Carlson = calcAvgRMS(globalVelTrue, globalVelCarlson);
velRMS_Takasu_mejorado = calcAvgRMS(globalVelTrue, globalVelTakasu_mejorado);
velRMS_Accel_only = calcAvgRMS(globalVelTrue, globalVelAccel_only);

% Error Acumulado de Velocidad
velAccum_EKF = calcAvgAccumError(globalVelTrue, globalVelEKF);
velAccum_Gyr = calcAvgAccumError(globalVelTrue, globalVelGyr);
velAccum_KF = calcAvgAccumError(globalVelTrue, globalVelKF);
velAccum_Matlab = calcAvgAccumError(globalVelTrue, globalVelMatlab);
velAccum_UDU = calcAvgAccumError(globalVelTrue, globalVelUDU);
velAccum_Takasu = calcAvgAccumError(globalVelTrue, globalVelTakasu);
velAccum_Carlson = calcAvgAccumError(globalVelTrue, globalVelCarlson);
velAccum_Takasu_mejorado = calcAvgAccumError(globalVelTrue, globalVelTakasu_mejorado);
velAccum_Accel_only = calcAvgAccumError(globalVelTrue, globalVelAccel_only);

% 3. Errores de Posición Promedio
posRMS_EKF = calcAvgRMS(globalPosTrue, globalPosEKF);
posRMS_Gyr = calcAvgRMS(globalPosTrue, globalPosGyr);
posRMS_KF = calcAvgRMS(globalPosTrue, globalPosKF);
posRMS_Matlab = calcAvgRMS(globalPosTrue, estPosition);
posRMS_UDU = calcAvgRMS(globalPosTrue, globalPosUDU);
posRMS_Takasu = calcAvgRMS(globalPosTrue, globalPosTakasu);
posRMS_Carlson = calcAvgRMS(globalPosTrue, globalPosCarlson);
posRMS_Takasu_mejorado = calcAvgRMS(globalPosTrue, globalPosTakasu_mejorado);
posRMS_Accel_only = calcAvgRMS(globalPosTrue, globalPosAccel_only);

% Error Acumulado de Posición
posAccum_EKF = calcAvgAccumError(globalPosTrue, globalPosEKF);
posAccum_Gyr = calcAvgAccumError(globalPosTrue, globalPosGyr);
posAccum_KF = calcAvgAccumError(globalPosTrue, globalPosKF);
posAccum_Matlab = calcAvgAccumError(globalPosTrue, estPosition);
posAccum_UDU = calcAvgAccumError(globalPosTrue, globalPosUDU);
posAccum_Takasu = calcAvgAccumError(globalPosTrue, globalPosTakasu);
posAccum_Carlson = calcAvgAccumError(globalPosTrue, globalPosCarlson);
posAccum_Takasu_mejorado = calcAvgAccumError(globalPosTrue, globalPosTakasu_mejorado);
posAccum_Accel_only = calcAvgAccumError(globalPosTrue, globalPosAccel_only);

% 4. Errores de Orientación considerando la periodicidad de los ángulos
trueEuler = euler(trueOrientation, 'ZYX', 'frame');

% Función para calcular RMS angular promedio (considerando periodicidad)
calcAngularRMS = @(est, true) mean([
    sqrt(mean(calc_min_error(est(:,1), true(:,1)).^2)), ...
    sqrt(mean(calc_min_error(est(:,2), true(:,2)).^2)), ...
    sqrt(mean(calc_min_error(est(:,3), true(:,3)).^2))
]);

% Función para calcular error angular acumulado promedio (considerando periodicidad)
calcAngularAccum = @(est, true) mean([
    sum(calc_min_error(est(:,1), true(:,1))), ...
    sum(calc_min_error(est(:,2), true(:,2))), ...
    sum(calc_min_error(est(:,3), true(:,3)))
]);

% Cálculo de errores RMS angulares
eulerRMS_EKF = calcAngularRMS(eulerEKF, trueEuler);
eulerRMS_Gyr = calcAngularRMS(eulerGyr, trueEuler);
eulerRMS_KF = calcAngularRMS(eulerKF, trueEuler);
eulerRMS_Matlab = calcAngularRMS(eulerEst, trueEuler);
eulerRMS_UDU = calcAngularRMS(eulerUDU, trueEuler);
eulerRMS_Takasu = calcAngularRMS(eulerTakasu, trueEuler);
eulerRMS_Carlson = calcAngularRMS(eulerCarlson, trueEuler);
eulerRMS_Takasu_mejorado = calcAngularRMS(eulerTakasu_mejorado, trueEuler);
eulerRMS_Accel_only = calcAngularRMS(eulerAccel_only, trueEuler);

% Cálculo de errores acumulados angulares
eulerAccum_EKF = calcAngularAccum(eulerEKF, trueEuler);
eulerAccum_Gyr = calcAngularAccum(eulerGyr, trueEuler);
eulerAccum_KF = calcAngularAccum(eulerKF, trueEuler);
eulerAccum_Matlab = calcAngularAccum(eulerEst, trueEuler);
eulerAccum_UDU = calcAngularAccum(eulerUDU, trueEuler);
eulerAccum_Takasu = calcAngularAccum(eulerTakasu, trueEuler);
eulerAccum_Carlson = calcAngularAccum(eulerCarlson, trueEuler);
eulerAccum_Takasu_mejorado = calcAngularAccum(eulerTakasu_mejorado, trueEuler);
eulerAccum_Accel_only = calcAngularAccum(eulerAccel_only, trueEuler);

%% 10. Presentación de Resultados
% Esta sección muestra en la consola los resultados cuantitativos calculados
% Se presentan los errores RMS y acumulados para cada variable y cada filtro

fprintf('\n============ RESULTADOS DE ERROR RMS PROMEDIO ============\n');

fprintf('\n1. Error RMS de Aceleración (m/s²):\n');
fprintf('                RMS     Acumulado\n');
fprintf('EKF:           %.4f  %.4f\n', accRMS_EKF, accAccum_EKF);
fprintf('Gyro:          %.4f  %.4f\n', accRMS_Gyr, accAccum_Gyr);
fprintf('KF:            %.4f  %.4f\n', accRMS_KF, accAccum_KF);
fprintf('Matlab:        %.4f  %.4f\n', accRMS_Matlab, accAccum_Matlab);
fprintf('UDU:           %.4f  %.4f\n', accRMS_UDU, accAccum_UDU);
fprintf('Takasu:        %.4f  %.4f\n', posRMS_Takasu, posAccum_Takasu);
fprintf('Carlson:       %.4f  %.4f\n', posRMS_Carlson, posAccum_Carlson);
fprintf('Takasu Mej:    %.4f  %.4f\n', posRMS_Takasu_mejorado, posAccum_Takasu_mejorado);
fprintf('Solo Accel:    %.4f  %.4f\n', posRMS_Accel_only, posAccum_Accel_only);

fprintf('\n4. Error RMS de Orientación (radianes):\n');
fprintf('                RMS     Acumulado\n');
fprintf('EKF:           %.4f  %.4f\n', eulerRMS_EKF, eulerAccum_EKF);
fprintf('Gyro:          %.4f  %.4f\n', eulerRMS_Gyr, eulerAccum_Gyr);
fprintf('KF:            %.4f  %.4f\n', eulerRMS_KF, eulerAccum_KF);
fprintf('Matlab:        %.4f  %.4f\n', eulerRMS_Matlab, eulerAccum_Matlab);
fprintf('UDU:           %.4f  %.4f\n', eulerRMS_UDU, eulerAccum_UDU);
fprintf('Takasu:        %.4f  %.4f\n', eulerRMS_Takasu, eulerAccum_Takasu);
fprintf('Carlson:       %.4f  %.4f\n', eulerRMS_Carlson, eulerAccum_Carlson);
fprintf('Takasu Mej:    %.4f  %.4f\n', eulerRMS_Takasu_mejorado, eulerAccum_Takasu_mejorado);
fprintf('Solo Accel:    %.4f  %.4f\n', eulerRMS_Accel_only, eulerAccum_Accel_only);

%% 11. Análisis de Tiempo de Ejecución
% Esta sección calcula y muestra el tiempo promedio de ejecución de cada filtro
% Valores menores indican mejor eficiencia computacional

fprintf('\n5. Tiempo Promedio de Ejecución (ms):\n');
fprintf('EKF:           %.2f\n', mean(timeEKF)*1000);
fprintf('Gyro:          %.2f\n', mean(timeGyr)*1000);
fprintf('KF:            %.2f\n', mean(timeKF)*1000);
fprintf('Matlab:        %.2f\n', mean(timeRef)*1000);
fprintf('UDU:           %.2f\n', mean(timeUDU)*1000);
fprintf('Takasu:        %.2f\n', mean(timeTakasu)*1000);
fprintf('Carlson:       %.2f\n', mean(timeCarlson)*1000);
fprintf('Takasu Mej:    %.2f\n', mean(timeTakasu_mejorado)*1000);
fprintf('Solo Accel:    %.2f\n', mean(timeAccel_only)*1000);

fprintf('\n2. Error RMS de Velocidad (m/s):\n');
fprintf('                RMS     Acumulado\n');
fprintf('EKF:           %.4f  %.4f\n', velRMS_EKF, velAccum_EKF);
fprintf('Gyro:          %.4f  %.4f\n', velRMS_Gyr, velAccum_Gyr);
fprintf('KF:            %.4f  %.4f\n', velRMS_KF, velAccum_KF);
fprintf('Matlab:        %.4f  %.4f\n', velRMS_Matlab, velAccum_Matlab);
fprintf('UDU:           %.4f  %.4f\n', velRMS_UDU, velAccum_UDU);
fprintf('Takasu:        %.4f  %.4f\n', velRMS_Takasu, velAccum_Takasu);
fprintf('Carlson:       %.4f  %.4f\n', velRMS_Carlson, velAccum_Carlson);
fprintf('Takasu Mej:    %.4f  %.4f\n', velRMS_Takasu_mejorado, velAccum_Takasu_mejorado);
fprintf('Solo Accel:    %.4f  %.4f\n', velRMS_Accel_only, velAccum_Accel_only);

fprintf('\n3. Error RMS de Posición (m):\n');
fprintf('                RMS     Acumulado\n');
fprintf('EKF:           %.4f  %.4f\n', posRMS_EKF, posAccum_EKF);
fprintf('Gyro:          %.4f  %.4f\n', posRMS_Gyr, posAccum_Gyr);
fprintf('KF:            %.4f  %.4f\n', posRMS_KF, posAccum_KF);
fprintf('Matlab:        %.4f  %.4f\n', posRMS_Matlab, posAccum_Matlab);
fprintf('UDU:           %.4f  %.4f\n', posRMS_UDU, posAccum_UDU);
fprintf('Takasu:        %.4f  %.4f\n', posRMS_Takasu, posAccum_Takasu);
fprintf('Carlson:       %.4f  %.4f\n', posRMS_Carlson, posAccum_Carlson);
fprintf('Takasu Mej:    %.4f  %.4f\n', posRMS_Takasu_mejorado, posAccum_Takasu_mejorado);
fprintf('Solo Accel:    %.4f  %.4f\n', posRMS_Accel_only, posAccum_Accel_only);

fprintf('\n4. Error RMS de Orientación (radianes):\n');
fprintf('                RMS     Acumulado\n');
fprintf('EKF:           %.4f  %.4f\n', eulerRMS_EKF, eulerAccum_EKF);
fprintf('Gyro:          %.4f  %.4f\n', eulerRMS_Gyr, eulerAccum_Gyr);
fprintf('KF:            %.4f  %.4f\n', eulerRMS_KF, eulerAccum_KF);
fprintf('Matlab:        %.4f  %.4f\n', eulerRMS_Matlab, eulerAccum_Matlab);
fprintf('UDU:           %.4f  %.4f\n', eulerRMS_UDU, eulerAccum_UDU);
fprintf('Takasu:        %.4f  %.4f\n', eulerRMS_Takasu, eulerAccum_Takasu);
fprintf('Carlson:       %.4f  %.4f\n', eulerRMS_Carlson, eulerAccum_Carlson);
fprintf('Takasu Mej:    %.4f  %.4f\n', eulerRMS_Takasu_mejorado, eulerAccum_Takasu_mejorado);
fprintf('Solo Accel:    %.4f  %.4f\n', eulerRMS_Accel_only, eulerAccum_Accel_only);

%% 11. Análisis de Tiempo de Ejecución
% Esta sección calcula y muestra el tiempo promedio de ejecución de cada filtro
% Valores menores indican mejor eficiencia computacional

fprintf('\n5. Tiempo Promedio de Ejecución (ms):\n');
fprintf('EKF:           %.2f\n', mean(timeEKF)*1000);
fprintf('Gyro:          %.2f\n', mean(timeGyr)*1000);
fprintf('KF:            %.2f\n', mean(timeKF)*1000);
fprintf('Matlab:        %.2f\n', mean(timeRef)*1000);
fprintf('UDU:           %.2f\n', mean(timeUDU)*1000);
fprintf('Takasu:        %.2f\n', mean(timeTakasu)*1000);
fprintf('Carlson:       %.2f\n', mean(timeCarlson)*1000);
fprintf('Takasu Mej:    %.2f\n', mean(timeTakasu_mejorado)*1000);
fprintf('Solo Accel:    %.2f\n', mean(timeAccel_only)*1000);

%% 12. Interpretación y Conclusiones
% Esta sección puede ser completada con una interpretación de los resultados
% y conclusiones sobre el rendimiento de los diferentes filtros.
%
% Algunos aspectos a considerar:
% - Precisión en estimación de orientación (ángulos de Euler)
% - Precisión en estimación de posición
% - Velocidad de ejecución
% - Complejidad de implementación
% - Robustez ante ruido y perturbaciones
%
% Un análisis más profundo puede incluir:
% - Comparativa de desempeño por escenarios (movimiento lento vs. rápido)
% - Dependencia del rendimiento con respecto a parámetros del filtro
% - Recomendaciones para aplicaciones específicas (drones, robótica, etc.)
%
% El mejor filtro dependerá de los requisitos específicos de la aplicación:
% - Aplicaciones en tiempo real pueden priorizar velocidad sobre precisión
% - Aplicaciones críticas pueden requerir mayor precisión a costa de procesamiento
% - Sistemas con recursos limitados pueden preferir algoritmos más simples

% Fin del script