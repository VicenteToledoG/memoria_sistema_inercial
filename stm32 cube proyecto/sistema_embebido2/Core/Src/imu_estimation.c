#include <math.h>
#include <string.h>
#include <assert.h>
#include "imu_estimation.h"
#include "linalg.h"  // Para las funciones matmul, etc.
#include "kalman_takasu.h"

// Funciones auxiliares para conversión de ángulos de Euler
#define MAT3(A, i, j) A[(i)*3 + (j)]

static void eul2rotm(const float* eul, float* R) {
    float phi = eul[0];   // roll  - rotación alrededor de X
    float theta = eul[1]; // pitch - rotación alrededor de Y
    float psi = eul[2];   // yaw   - rotación alrededor de Z

    // Matrices de rotación elementales como arrays 3x3
    float Rx[9] = {
        1.0f,      0.0f,       0.0f,
        0.0f, cosf(phi), -sinf(phi),
        0.0f, sinf(phi),  cosf(phi)
    };

    float Ry[9] = {
         cosf(theta), 0.0f, sinf(theta),
                0.0f, 1.0f,        0.0f,
        -sinf(theta), 0.0f, cosf(theta)
    };

    float Rz[9] = {
        cosf(psi), -sinf(psi), 0.0f,
        sinf(psi),  cosf(psi), 0.0f,
             0.0f,       0.0f, 1.0f
    };

    // Calcular R = Rz * Ry * Rx
    float temp[9];
    matmul("N", "N", 3, 3, 3, 1.0f, Rz, Ry, 0.0f, temp);
    matmul("N", "N", 3, 3, 3, 1.0f, temp, Rx, 0.0f, R);
}

static void rotm2eul(const float* R, float* eul) {
    // Umbral para gimbal lock
    const float GIMBAL_LOCK_THRESHOLD = 0.999999f;

    // Extraer ángulos de Euler
    // Manejo especial para gimbal lock
    if (fabsf(MAT3(R,0,2)) >= GIMBAL_LOCK_THRESHOLD) {  // R(1,3) en MATLAB es R[0,2] en C
        // Gimbal lock en theta = ±90°
        eul[2] = 0.0f;  // yaw (psi)

        if (MAT3(R,0,2) < 0) {  // R(1,3) < 0
            eul[1] = M_PI_2;  // pitch (theta)
            eul[0] = eul[2] + atan2f(MAT3(R,1,0), MAT3(R,1,1));  // roll (phi)
        } else {
            eul[1] = -M_PI_2;
            eul[0] = -eul[2] + atan2f(-MAT3(R,1,0), -MAT3(R,1,1));
        }
    } else {
        eul[1] = -asinf(MAT3(R,0,2));  // pitch (theta)
        float cos_theta = cosf(eul[1]);

        // Evitar división por números muy pequeños
        if (cos_theta > 1e-10f) {
            eul[0] = atan2f(MAT3(R,1,2)/cos_theta, MAT3(R,2,2)/cos_theta);  // roll (phi)
            eul[2] = atan2f(MAT3(R,0,1)/cos_theta, MAT3(R,0,0)/cos_theta);  // yaw (psi)
        } else {
            // Si cos_theta es muy pequeño, podemos tener problemas numéricos
            // En este caso, podemos usar un método alternativo o devolver valores por defecto
            eul[0] = 0.0f;
            eul[2] = atan2f(MAT3(R,1,0), MAT3(R,1,1));
        }
    }
}

void EulerAccel(float ax, float ay, float az, float* phi_a, float* theta_a) {
    *theta_a = atan2f(-ax, sqrtf(ay * ay + az * az));
    *phi_a = atan2f(ay, az);
}

void initEulerTakasuState(EulerTakasuState* state, float phi_i, float theta_i, float psi_i) {
    // Inicializar estado
    state->x[0] = phi_i;
    state->x[1] = theta_i;
    state->x[2] = psi_i;

    // Inicializar P = 10*I
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            state->P[i][j] = (i == j) ? 10.0f : 0.0f;
        }
    }

    // Inicializar H
    state->H[0][0] = 1.0f; state->H[0][1] = 0.0f; state->H[0][2] = 0.0f;
    state->H[1][0] = 0.0f; state->H[1][1] = 1.0f; state->H[1][2] = 0.0f;

    // Inicializar Q
    state->Q[0][0] = 0.0001f; state->Q[0][1] = 0.0f;    state->Q[0][2] = 0.0f;
    state->Q[1][0] = 0.0f;    state->Q[1][1] = 0.0001f; state->Q[1][2] = 0.0f;
    state->Q[2][0] = 0.0f;    state->Q[2][1] = 0.0f;    state->Q[2][2] = 0.1f;

    // Inicializar R
    state->R[0][0] = 6.0f; state->R[0][1] = 0.0f;
    state->R[1][0] = 0.0f; state->R[1][1] = 6.0f;

    state->initialized = 1;
}

void EulerTakasu(float* z, float* gyroData, float dt,
                 float phi_i, float theta_i, float psi_i,
                 EulerTakasuState* state,
                 float* phi_out, float* theta_out, float* psi_out) {

    const float kp = 0.004f;  // Ganancia proporcional igual que en MATLAB

    if (!state->initialized) {
        initEulerTakasuState(state, phi_i, theta_i, psi_i);
    }

    // Aplanar matrices para kalman_takasu
    float P_flat[9], Ht_flat[6], R_flat[4];

    // Convertir de formato matriz a array plano en columna-mayor
    for (int j = 0; j < 3; j++) {
        for (int i = 0; i < 3; i++) {
            P_flat[j*3 + i] = state->P[i][j];
        }
    }
    for (int j = 0; j < 3; j++) {
        for (int i = 0; i < 2; i++) {
            Ht_flat[i*3 + j] = state->H[i][j];
        }
    }
    for (int j = 0; j < 2; j++) {
        for (int i = 0; i < 2; i++) {
            R_flat[j*2 + i] = state->R[i][j];
        }
    }

    // Predicción
    float xp[3];
    fx(xp, state->x, gyroData, dt);

    float A[9];
    Ajacob(A, state->x, gyroData, dt);

    // Calcular dz = z - H*xp
    float Hx[3] = {0};
    float dz[3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            Hx[i] += state->H[i][j] * xp[j];
        }
        dz[i] = z[i] - Hx[i];
    }

    // Ajustar dz[2] a 0 explícitamente
    dz[2] = 0.0f;
    dz[1]=dz[1]; // *cos(xp[1]);  //ayuda pero no es la solucion del gimbal lock
    // Ajustar ángulos de Euler
    adjustEulerAngles(dz);

    // Aplicar Kalman Takasu usando H transpuesta
    float chi2;
    int result = kalman_takasu(xp, P_flat, dz, R_flat, Ht_flat, 3, 2, 0.0f, &chi2);

    if (result == 0) {
        // Actualizar estado y matriz P
        for (int j = 0; j < 3; j++) {
            for (int i = 0; i < 3; i++) {
                state->P[i][j] = P_flat[j*3 + i];
            }
        }

        // Aplicar corrección proporcional a roll y pitch
        xp[0] += kp * dz[0]; // *cos(xp[1]);  // ayuda pero no es solucion del gimbal lock
        xp[1] += kp * dz[1];  // Corrección de pitch

        // Actualizar estado
        for (int i = 0; i < 3; i++) {
            state->x[i] = xp[i];
        }

        // Manejar gimbal lock
        float eul[3] = {xp[0], xp[1], xp[2]};
        float R[9];
        eul2rotm(eul, R);
        rotm2eul(R, eul);

        *phi_out = xp[0];
        *theta_out = xp[1];  //xp se debe reemplazar por eul, solucion temporal
        *psi_out = xp[2];
    }
}

// Función para multiplicar matrices de rotación usando la función matmul existente
void multRotationMatrices(const float* A, const float* B, float* C) {
    matmul("N", "N", 3, 3, 3, 1.0f, A, B, 0.0f, C);
}

// Función para multiplicar matriz de rotación por vector usando matmul
void rotateVector(const float* R, const float* v, float* result) {
    matmul("N", "N", 3, 1, 3, 1.0f, R, v, 0.0f, result);
}

// Función para inicializar el estado del sistema
void initIMUState(IMUState* state, float dt) {
    memset(state, 0, sizeof(IMUState));
    state->dt = dt;

    // Inicializar matrices de covarianza
    float* P = (float*)state->takasu_state.P;
    mateye(P, 3);
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            P[i*3 + j] *= 10.0f;
        }
        state->prevVelGlobal[i] = 0.0f;
        state->prevPosGlobal[i] = 0.0f;
        state->accelGlobal[i] = 0.0f;
    }
}

// Implementación mejorada de transformAccelToGlobal usando las funciones existentes
void transformAccelToGlobal(const float* eulerAngles, const float* accelData,
                           float dt, float* prevVelGlobal, float* prevPosGlobal,
                           float* accelGlobal, float* velGlobal, float* posGlobal) {
    float yaw = eulerAngles[0];
    float pitch = eulerAngles[1];
    float roll = eulerAngles[2];

    // Construir matrices de rotación
    float Rz[9] = {
        cosf(yaw), -sinf(yaw), 0,
        sinf(yaw), cosf(yaw),  0,
        0,         0,          1
    };
    float Ry[9] = {
        cosf(pitch),  0, sinf(pitch),
        0,           1, 0,
        -sinf(pitch), 0, cosf(pitch)
    };
    float Rx[9] = {
        1, 0,          0,
        0, cosf(roll), -sinf(roll),
        0, sinf(roll), cosf(roll)
    };

    // Calcular matriz de rotación completa usando las funciones existentes
    float temp[9], R[9];
    multRotationMatrices(Rz, Ry, temp);
    multRotationMatrices(temp, Rx, R);

    // Transformar aceleración al marco global usando la función existente
    rotateVector(R, accelData, accelGlobal);

    // Compensar gravedad
    accelGlobal[2] += 9.81f;

    // Integrar velocidad y posición
    for (int i = 0; i < 3; i++) {
        velGlobal[i] = prevVelGlobal[i] + accelGlobal[i] * dt;
        posGlobal[i] = prevPosGlobal[i] + velGlobal[i] * dt;
    }
}

// Función principal para procesar datos de IMU
void processIMUData(IMUState* state,
                   float accX, float accY, float accZ,
                   float gyrX, float gyrY, float gyrZ,
                   float* phi_out, float* theta_out, float* psi_out,
                   float* pos_out, float* vel_out) {

    // Datos de acelerómetro y giroscopio
    float accelData[3] = {-accX, -accY, -accZ};
    float gyroData[3] = {gyrX, gyrY, gyrZ};

    // Calcular ángulos iniciales desde acelerómetro
    float phi_a, theta_a;
    EulerAccel(accX, accY, accZ, &phi_a, &theta_a);

    // Entrada para el filtro Takasu
    float z[3] = {phi_a, theta_a,0};

    // Aplicar filtro Takasu
    float phi, theta, psi;
    EulerTakasu(z, gyroData, state->dt, phi_a, theta_a, 0,
                &state->takasu_state, &phi, &theta, &psi);

    // Guardar ángulos de salida
    *phi_out = phi;
    *theta_out = theta;
    *psi_out = psi;

    // Convertir aceleración al marco global y actualizar posición/velocidad
    float eulerAngles[3] = {psi, theta, phi};  // ZYX order
	float velGlobal[3], posGlobal[3];

	transformAccelToGlobal(eulerAngles, accelData, state->dt,
						  state->prevVelGlobal, state->prevPosGlobal,
						  state->accelGlobal,  // Guardamos directamente en state
						  velGlobal, posGlobal);

    // Actualizar estados previos
    memcpy(state->prevVelGlobal, velGlobal, sizeof(velGlobal));
    memcpy(state->prevPosGlobal, posGlobal, sizeof(posGlobal));

    // Copiar resultados a las salidas
    memcpy(vel_out, velGlobal, sizeof(float)*3);
    memcpy(pos_out, posGlobal, sizeof(float)*3);
}

// Función para ajustar ángulos de Euler dentro del rango [-pi, pi]
void adjustEulerAngles(float* angles) {
    for (int i = 0; i < 3; i++) {
        while (angles[i] > M_PI) angles[i] -= 2 * M_PI;
        while (angles[i] < -M_PI) angles[i] += 2 * M_PI;
    }
}

// Predicción del estado xp
void fx(float* xp, float* xhat, float* rates, float dt) {
    float phi = xhat[0];
    float theta = xhat[1];
    float p = rates[0];
    float q = rates[1];
    float r = rates[2];

    // Derivadas de los estados
    float xdot[3];
    xdot[0] = p + q * sin(phi) * tan(theta) + r * cos(phi) * tan(theta);
    xdot[1] = q * cos(phi) - r * sin(phi);
    xdot[2] = q * sin(phi) / cos(theta) + r * cos(phi) / cos(theta);

    // Actualizar xp = xhat + xdot * dt
    for (int i = 0; i < 3; i++) {
        xp[i] = xhat[i] + xdot[i] * dt;
    }

    // Ajustar ángulos de Euler
    adjustEulerAngles(xp);
}

// Cálculo de la matriz Jacobiana A
void Ajacob(float* A, float* xhat, float* rates, float dt) {
    float phi = xhat[0];
    float theta = xhat[1];
    //float p = rates[0];
    float q = rates[1];
    float r = rates[2];

    // Rellenar la matriz A con los valores calculados
    A[0] = 1 + (q * cos(phi) * tan(theta) - r * sin(phi) * tan(theta)) * dt;
    A[1] = (q * sin(phi) / (cos(theta) * cos(theta)) + r * cos(phi) / (cos(theta) * cos(theta))) * dt;
    A[2] = 0;
    A[3] = (-q * sin(phi) - r * cos(phi)) * dt;
    A[4] = 1;
    A[5] = 0;
    A[6] = (q * cos(phi) / cos(theta) - r * sin(phi) / cos(theta)) * dt;
    A[7] = (q * sin(phi) * tan(theta) / cos(theta) + r * cos(phi) * tan(theta) / cos(theta)) * dt;
    A[8] = 1;
}
