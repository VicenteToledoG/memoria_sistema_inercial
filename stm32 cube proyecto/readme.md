# Sistema de Estimación de Orientación con IMU

Este proyecto implementa un sistema de estimación de orientación (ángulos de Euler) utilizando un sensor inercial MPU6050 en una placa STM32 Nucleo-F4. El sistema captura datos del acelerómetro y giroscopio, procesa la información para estimar la orientación en tiempo real, y envía los resultados a través de UART.

## Descripción General

El sistema:
- Lee datos del sensor IMU MPU6050 mediante comunicación I2C
- Procesa los datos para estimar ángulos de Euler (roll, pitch, yaw)
- Transmite los ángulos calculados a través de UART para visualización o análisis posterior
- Funciona a una frecuencia de muestreo de 2500 Hz con transmisión a 125 Hz

## Hardware Requerido

- Placa STM32 Nucleo-F4 (compatible con STM32F401RE, STM32F411RE, etc.)
- Sensor IMU MPU6050
- Conexión USB para programación y comunicación serial

## Conexiones

| MPU6050 | STM32 Nucleo-F4 |
|---------|-----------------|
| VCC     | 3.3V            |
| GND     | GND             |
| SCL     | PB8 (I2C1_SCL)  |
| SDA     | PB9 (I2C1_SDA)  |
| INT     | No utilizado    |

## Requisitos de Software

- STM32CubeIDE (versión 1.9.0 o superior)
- Biblioteca HAL de STM para STM32F4
- Librerías adicionales incluidas en el proyecto:
  - `linalg.h/.c`: Operaciones de álgebra lineal
  - `miniblas.h/.c`: Funciones para manipulación de matrices
  - `imu_estimation.h/.c`: Algoritmos de estimación para IMU

## Configuración del Proyecto

1. Clonar o descargar este repositorio
2. Abrir STM32CubeIDE
3. Seleccionar File > Open Projects from File System...
4. Navegar hasta la carpeta del proyecto y seleccionarla
5. Hacer clic en "Finish" para importar el proyecto

## Estructura del Proyecto

```
├── Core/
│   ├── Inc/                  # Archivos de cabecera
│   │   ├── main.h            # Cabecera principal
│   │   ├── linalg.h          # Operaciones de álgebra lineal
│   │   ├── miniblas.h        # Funciones para matrices
│   │   └── imu_estimation.h  # Algoritmos de estimación
│   └── Src/                  # Archivos fuente
│       ├── main.c            # Programa principal
│       ├── linalg.c          # Implementación de álgebra lineal
│       ├── miniblas.c        # Implementación de funciones de matrices
│       └── imu_estimation.c  # Implementación de algoritmos
├── Drivers/                  # Controladores STM32
└── [otros archivos de configuración]
```

## Compilación y Programación

1. Conectar la placa STM32 Nucleo-F4 al ordenador mediante USB
2. En STM32CubeIDE, hacer clic derecho en el proyecto y seleccionar "Build Project"
3. Una vez compilado correctamente, hacer clic derecho y seleccionar "Run As > STM32 C/C++ Application"
4. El programa se cargará en la placa y comenzará a ejecutarse

## Protocolo de Comunicación

El sistema envía los ángulos de Euler calculados a través de UART con la siguiente configuración:
- Velocidad: 115200 baudios
- Bits de datos: 8
- Paridad: Ninguna
- Bits de parada: 1
- Control de flujo: Ninguno

### Formato de los Datos

Cada paquete de datos enviado consta de 13 bytes:
1. 1 byte de sincronización (0xAA)
2. 4 bytes para el ángulo phi (roll) - formato float
3. 4 bytes para el ángulo theta (pitch) - formato float
4. 4 bytes para el ángulo psi (yaw) - formato float

Los valores float se envían en orden de bytes invertido (little-endian).

## Recepción y Visualización de Datos

Para visualizar los datos recibidos, puede utilizar:
- Un terminal serial como PuTTY, Tera Term o la terminal integrada en STM32CubeIDE
- Software personalizado para capturar y procesar los paquetes de datos
- Herramientas como Processing o MATLAB para visualización gráfica

## Calibración del Sensor

El código incluye funciones de calibración para el acelerómetro y giroscopio:
- `convertAcc()`: Convierte lecturas crudas del acelerómetro a m/s²
- `convertGyro()`: Convierte lecturas crudas del giroscopio a rad/s

Los parámetros de calibración actuales están configurados para un sensor específico. Para mejores resultados, deberá ajustar estos valores para su propio sensor MPU6050.

## Personalización

### Modificar la Frecuencia de Muestreo

Para cambiar la frecuencia de muestreo:
1. Modificar la configuración del Timer1 en la función `MX_TIM1_Init()`
2. Actualizar la variable `dt` en el código para que coincida con el nuevo período

### Modificar la Frecuencia de Transmisión

Para cambiar la frecuencia con la que se envían los paquetes de datos:
1. Ajustar la condición `if(sample_counter >= 20)` en el bucle principal
   - Aumentar el valor para reducir la frecuencia de transmisión
   - Disminuir el valor para aumentar la frecuencia de transmisión

## Solución de Problemas

### El sensor MPU6050 no responde
- Verificar las conexiones I2C (SCL y SDA)
- Confirmar que la alimentación del sensor sea adecuada (3.3V)
- Comprobar la dirección I2C del dispositivo (generalmente 0x68)

### No se reciben datos por UART
- Verificar la configuración del puerto serial en la aplicación de recepción
- Comprobar que la velocidad y otros parámetros coincidan con la configuración del código
- Asegurar que los cables de comunicación UART estén correctamente conectados
