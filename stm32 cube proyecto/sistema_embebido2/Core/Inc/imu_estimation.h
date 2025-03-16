#ifndef IMU_ESTIMATION_H
#define IMU_ESTIMATION_H

#ifdef __cplusplus
extern "C" {
#endif

// Estructura para el estado del filtro Takasu
typedef struct {
    float x[3];           // Estado [phi, theta, psi]
    float P[3][3];       // Matriz de covarianza
    float H[2][3];       // Matriz de observación
    float Q[3][3];       // Matriz de covarianza del proceso
    float R[2][2];       // Matriz de covarianza de la medición
    int initialized;     // Flag de inicialización
} EulerTakasuState;

// Estructura para el estado completo del sistema IMU
typedef struct {
    EulerTakasuState takasu_state;
    float prevVelGlobal[3];
    float prevPosGlobal[3];
    float accelGlobal[3];     // Agregamos esto para almacenar la aceleración global
    float dt;
} IMUState;

// Funciones principales
void EulerAccel(float ax, float ay, float az, float* phi_a, float* theta_a);

void EulerTakasu(float* z, float* gyroData, float dt,
                 float phi_i, float theta_i, float psi_i,
                 EulerTakasuState* state,
                 float* phi_out, float* theta_out, float* psi_out);

void transformAccelToGlobal(const float* eulerAngles, const float* accelData,
                           float dt, float* prevVelGlobal, float* prevPosGlobal,
                           float* accelGlobal, float* velGlobal, float* posGlobal);

void processIMUData(IMUState* state,
                   float accX, float accY, float accZ,
                   float gyrX, float gyrY, float gyrZ,
                   float* phi_out, float* theta_out, float* psi_out,
                   float* pos_out, float* vel_out);

// Funciones de inicialización
void initEulerTakasuState(EulerTakasuState* state, float phi_i, float theta_i, float psi_i);
void initIMUState(IMUState* state, float dt);

// Función de predicción de estado
void fx(float* xp, float* xhat, float* rates, float dt);

// Cálculo de la matriz Jacobiana A
void Ajacob(float* A, float* xhat, float* rates, float dt);

// Ajusta los ángulos de Euler para que estén en el rango [-pi, pi]
void adjustEulerAngles(float* angles);

#ifdef __cplusplus
}
#endif

#endif // IMU_ESTIMATION_H
