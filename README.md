# Sistema de Estimación de Orientación y Posición basado en IMU

Este repositorio contiene un sistema completo para la estimación de orientación y posición utilizando Unidades de Medición Inercial (IMU), dividido en dos componentes principales:

1. **Implementación en hardware** - Sistema embebido en STM32 Nucleo-F4 para estimación en tiempo real
2. **Simulación y análisis** - Scripts de MATLAB para evaluar diferentes algoritmos de fusión sensorial

El proyecto permite tanto realizar simulaciones y análisis comparativos de diferentes algoritmos de fusión sensorial en MATLAB, como implementar estos algoritmos en un sistema embebido real basado en un microcontrolador STM32.

## Visión General del Proyecto

El sistema integra:
- Adquisición de datos de sensores inerciales (acelerómetro y giroscopio)
- Implementación de diversos algoritmos de fusión sensorial
- Estimación de ángulos de Euler (orientación), velocidad y posición
- Evaluación comparativa de diferentes filtros
- Implementación en tiempo real en hardware STM32

## Estructura del Repositorio

```
├── Hardware/                        # Implementación en STM32
│   ├── Core/                        # Código fuente del sistema embebido
│   │   ├── Inc/                     # Archivos de cabecera
│   │   └── Src/                     # Archivos fuente
│   ├── Drivers/                     # Controladores STM32
│   └── [otros archivos STM32CubeIDE]
│
└── MATLAB/                         # Simulación y análisis
    ├── comparacion_filtros.m       # Script de simulación con datos sintéticos
    ├── prueba_filtros_real.m       # Script para procesar datos reales
    ├── GyroscopeIntegration/       # Algoritmos de integración de giroscopio
    ├── AccelerometerMagnetometer/  # Algoritmos para datos de acelerómetro
    ├── EulerKF/                    # Implementación de Filtro de Kalman
    ├── EulerEKF/                   # Implementación de Filtro de Kalman Extendido
    ├── EulerUKF/                   # Implementación de Filtro de Kalman Unscented
    ├── Functions/                   # Funciones auxiliares
    └── *.mat                        # Archivos de datos reales para pruebas
```

## Parte 1: Implementación en Hardware (STM32)

### Descripción

El sistema embebido implementa algoritmos de fusión sensorial en una placa STM32 Nucleo-F4 que se comunica con un sensor IMU MPU6050 para estimar la orientación en tiempo real y transmitir los resultados mediante UART.

### Hardware Requerido

- Placa STM32 Nucleo-F4 (compatible con STM32F401RE, STM32F411RE, etc.)
- Sensor IMU MPU6050
- Conexión USB para programación y comunicación serial

### Conexiones

| MPU6050 | STM32 Nucleo-F4 |
|---------|-----------------|
| VCC     | 3.3V            |
| GND     | GND             |
| SCL     | PB8 (I2C1_SCL)  |
| SDA     | PB9 (I2C1_SDA)  |
| INT     | No utilizado    |

### Requisitos de Software

- STM32CubeIDE (versión 1.9.0 o superior)
- Biblioteca HAL de STM para STM32F4

### Configuración e Instalación

1. Clonar o descargar este repositorio
2. Abrir STM32CubeIDE
3. Importar el proyecto desde la carpeta Hardware/
4. Compilar y cargar el programa en la placa STM32

### Protocolo de Comunicación

El sistema envía los ángulos de Euler calculados a través de UART (115200 baudios, 8N1) con el siguiente formato:
- 1 byte de sincronización (0xAA)
- 4 bytes para el ángulo phi (roll) - formato float
- 4 bytes para el ángulo theta (pitch) - formato float
- 4 bytes para el ángulo psi (yaw) - formato float

## Parte 2: Simulación y Análisis (MATLAB)

### Descripción

La parte de MATLAB proporciona herramientas para evaluar y comparar diferentes algoritmos de fusión sensorial, tanto con datos simulados como con datos reales capturados de IMUs.

### Algoritmos Implementados

- Filtro de Referencia de MATLAB (insfilterNonholonomic)
- Filtro de Kalman Extendido (EKF)
- Integración de Giroscopio (Gyro)
- Filtro de Kalman Lineal (KF)
- Filtro UDU
- Filtro Takasu
- Filtro Carlson
- Filtro Takasu Mejorado
- Filtro Solo Acelerómetro (solo en simulación)

### Requisitos

- MATLAB (R2021b o posterior)
- MATLAB Navigation Toolbox

### Scripts Principales

- **comparacion_filtros.m**: Evalúa los filtros utilizando datos simulados de una trayectoria predefinida.
- **prueba_filtros_real.m**: Evalúa los filtros utilizando datos reales capturados de una IMU.

### Datos de Prueba

El repositorio incluye varios archivos .mat con datos reales capturados de IMUs:
- **Rotacion_3.mat** - Datos de rotación
- **Locura.mat** - Datos alternativos
- **prueba_dibujo.mat** - Datos alternativos

## Flujo de Trabajo Completo

El proyecto permite un flujo de trabajo completo:

1. **Fase de Investigación y Desarrollo**:
   - Usar los scripts de MATLAB para simular diferentes escenarios
   - Comparar el rendimiento de distintos algoritmos de fusión sensorial
   - Determinar el algoritmo óptimo para su caso de uso específico

2. **Fase de Implementación**:
   - Implementar el algoritmo seleccionado en el sistema embebido STM32
   - Ajustar parámetros como frecuencia de muestreo, calibración, etc.
   - Evaluar el rendimiento en tiempo real

3. **Fase de Validación**:
   - Capturar datos del sistema embebido
   - Analizar los resultados con las herramientas de MATLAB
   - Comparar con las simulaciones previas para validar el rendimiento

## Personalización

### Simulación en MATLAB

Puede modificar:
- Frecuencia de muestreo (`imuFs`)
- Parámetros de ruido de los sensores
- Parámetros de los filtros
- Trayectorias simuladas

### Implementación en Hardware

Puede ajustar:
- Frecuencia de muestreo (modificando la configuración del Timer)
- Parámetros de calibración del sensor
- Frecuencia de transmisión de datos
- Algoritmo de fusión implementado

## Aplicaciones Potenciales

Este sistema puede ser utilizado en varias aplicaciones:

- Sistemas de navegación inercial
- Estabilización de drones o vehículos
- Captura de movimiento
- Robótica
- Realidad virtual y aumentada
- Wearables para monitoreo de actividad física
