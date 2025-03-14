# Análisis Comparativo de Filtros de Fusión Sensorial para IMU

Este repositorio contiene scripts de MATLAB para evaluar y comparar diferentes algoritmos de fusión sensorial para IMUs (Unidades de Medición Inercial). Los algoritmos implementados estiman orientación, velocidad y posición a partir de datos de acelerómetro y giroscopio.

## Contenido

El repositorio contiene dos scripts principales:

- **comparacion_filtros.m**: Evalúa los filtros utilizando datos simulados de una trayectoria predefinida.
- **prueba_filtros_real.m**: Evalúa los filtros utilizando datos reales capturados de una IMU.

## Algoritmos Implementados

Los scripts implementan y comparan los siguientes filtros de fusión sensorial:

- Filtro de Referencia de MATLAB (insfilterNonholonomic)
- Filtro de Kalman Extendido (EKF)
- Integración de Giroscopio (Gyro)
- Filtro de Kalman Lineal (KF)
- Filtro UDU
- Filtro Takasu
- Filtro Carlson
- Filtro Takasu Mejorado
- Filtro Solo Acelerómetro (solo en simulación)

## Requisitos

- MATLAB (desarrollado y probado en MATLAB R2021b o posterior)
- MATLAB Navigation Toolbox (para el filtro de referencia insfilterNonholonomic)

## Estructura de Directorios
├── comparacion_filtros.m      # Script de simulación
├── prueba_filtros_real.m      # Script para datos reales
├── Rotacion_3.mat             # Archivo de datos reales (rotación)
├── Locura.mat                 # Archivo alternativo de datos reales
├── prueba_dibujo.mat          # Archivo alternativo de datos reales
├── GyroscopeIntegration/      # Funciones para integración de giroscopio
├── AccelerometerMagnetometer/ # Funciones para procesar datos de acelerómetro
├── EulerKF/                   # Implementación de Filtro de Kalman para ángulos de Euler
├── EulerEKF/                  # Implementación de Filtro de Kalman Extendido
├── EulerUKF/                  # Implementación de Filtro de Kalman Unscented
└── Functions/                 # Funciones auxiliares generales

## Uso del Script de Simulación

El script `comparacion_filtros.m` genera una trayectoria sintética circular con orientación variable y simula las lecturas de una IMU. Luego ejecuta cada uno de los filtros y compara sus resultados con el ground truth conocido.

Para ejecutar:

1. Asegúrese de que todos los directorios necesarios estén en el path de MATLAB.
2. Abra el script `comparacion_filtros.m` en MATLAB.
3. Ejecute el script completo.
4. Examine los gráficos generados y los resultados de error RMS mostrados en la consola.

## Uso del Script con Datos Reales

El script `prueba_filtros_real.m` utiliza datos capturados de una IMU real, los procesa y evalúa el rendimiento de los diferentes filtros.

Para ejecutar:

1. Asegúrese de que los archivos .mat con los datos reales estén disponibles (Rotacion_3.mat, Locura.mat, prueba_dibujo.mat).
2. Abra el script `prueba_filtros_real.m` en MATLAB.
3. Descomente la línea correspondiente al conjunto de datos que desea utilizar.
4. Ejecute el script completo.
5. Examine los gráficos comparativos y los tiempos de ejecución mostrados.

## Personalización

### Parámetros de Simulación

En `comparacion_filtros.m` puede modificar:
- Frecuencia de muestreo (`imuFs`)
- Parámetros de la trayectoria (`r`, `speed`, `numRevs`)
- Parámetros de ruido de los sensores

### Datos Reales

En `prueba_filtros_real.m` puede:
- Seleccionar diferentes conjuntos de datos (descomentando las líneas correspondientes)
- Ajustar los parámetros de calibración para los datos raw del sensor
- Modificar el número de muestras a procesar (`numsamples`)

## Métricas de Evaluación

Ambos scripts evalúan los filtros utilizando varias métricas:

- Error RMS (Root Mean Square) para aceleración, velocidad, posición y orientación
- Error acumulado para las mismas variables
- Tiempo de ejecución promedio de cada filtro

## Contribuciones

Este proyecto está abierto a contribuciones. Si desea agregar nuevos filtros o mejorar los existentes, por favor cree un pull request con sus cambios.

## Licencia

[Insertar información de licencia aquí]