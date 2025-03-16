/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */


#include "main.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "linalg.h" // Incluye tu implementación de operaciones de álgebra lineal
#include "miniblas.h" // Incluye tu implementación para cálculos de matrices
#include "imu_estimation.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
MPU6050_t MPU6050;
float accX, accY, accZ;
float gyrX, gyrY, gyrZ;
uint16_t sample_counter;
int int_counter;
uint8_t leer;

/* Variables para almacenar resultados */
float phi, theta, psi;           // Ángulos de Euler actuales
float accelGlobal[1][3];        // Aceleración en marco global
float velGlobal[1][3];          // Velocidad en marco global
float posGlobal[1][3];          // Posición en marco global

/* Variables de estado del sistema */
IMUState imu_state;
float dt = 0.0004f;             // Período de muestreo
/* USER CODE END PV */

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);

/* USER CODE BEGIN 0 */
float convertAcc(float raw, float maxRaw, float minRaw, float center) {
    if (raw >= center) {
        return (raw - center) * (9.81 / (maxRaw - center));
    } else {
        return (raw - center) * (9.81 / (center - minRaw));
    }
}

float convertGyro(float raw, float maxRaw, float minRaw, float center) {
    if (raw >= center) {
        return (raw - center) * (3.0 / (maxRaw - center));
    } else {
        return (raw - center) * (3.0 / (center - minRaw));
    }
}

void send_euler_angles(void) {
    uint8_t sync_byte = 0xAA;  // Byte de sincronización
    uint8_t bytes[4];

    // Enviar byte de sincronización
    HAL_UART_Transmit(&huart2, &sync_byte, 1, HAL_MAX_DELAY);

    // Enviar phi
    memcpy(bytes, &phi, sizeof(float));
    HAL_UART_Transmit(&huart2, &bytes[3], 1, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, &bytes[2], 1, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, &bytes[1], 1, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, &bytes[0], 1, HAL_MAX_DELAY);

    // Enviar theta
    memcpy(bytes, &theta, sizeof(float));
    HAL_UART_Transmit(&huart2, &bytes[3], 1, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, &bytes[2], 1, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, &bytes[1], 1, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, &bytes[0], 1, HAL_MAX_DELAY);

    // Enviar psi
    memcpy(bytes, &psi, sizeof(float));
    HAL_UART_Transmit(&huart2, &bytes[3], 1, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, &bytes[2], 1, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, &bytes[1], 1, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, &bytes[0], 1, HAL_MAX_DELAY);
}
/* USER CODE END 0 */

int main(void)
{
    /* USER CODE BEGIN 1 */
    sample_counter = 0;
    leer = 0;
    int_counter=0;

    /* USER CODE END 1 */

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
      HAL_Init();

      /* USER CODE BEGIN Init */

      /* USER CODE END Init */

      /* Configure the system clock */
      SystemClock_Config();

      /* USER CODE BEGIN SysInit */

      /* USER CODE END SysInit */

      /* Initialize all configured peripherals */
      MX_GPIO_Init();
      MX_I2C1_Init();
      MX_USART2_UART_Init();
      MX_TIM1_Init();
      /* USER CODE BEGIN 2 */
      while (MPU6050_Init(&hi2c1) == 1);
      if (HAL_TIM_Base_Start_IT(&htim1) != HAL_OK)
      {
        /* Starting Error */
        Error_Handler();
      }

      // Inicializar el estado del sistema IMU
      initIMUState(&imu_state, dt);
    /* USER CODE END 2 */

    /* Infinite loop */
    while (1)
    {
        if(leer) {
            leer = 0;
            MPU6050_Read_All(&hi2c1, &MPU6050);

            // Convertir lecturas crudas a unidades físicas
            accX = convertAcc(MPU6050.Accel_X_RAW, 17000, -16400, 0);
            accY = convertAcc(MPU6050.Accel_Y_RAW, 16400, -16500, 0);
            accZ = convertAcc(MPU6050.Accel_Z_RAW, 19350, -14600, 0);
            gyrX = convertGyro(MPU6050.Gyro_X_RAW, 7200, -6200, 625);
            gyrY = convertGyro(MPU6050.Gyro_Y_RAW, 6500, -7600, -463);
            gyrZ = convertGyro(MPU6050.Gyro_Z_RAW, 7600, -7600, 200);

            // Procesar datos IMU
            processIMUData(&imu_state,
                         accX, accY, accZ,
                         gyrX, gyrY, gyrZ,
                         &phi, &theta, &psi,
                         posGlobal[0],
                         velGlobal[0]);

            sample_counter++;

            // Enviar ángulos de Euler cada 100 muestras
            if(sample_counter >= 20) {
                send_euler_angles();
                sample_counter = 0;
            }
        }
    }
}



/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00702991;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 32;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 230400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD4_Pin */
  GPIO_InitStruct.Pin = LD4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD4_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
