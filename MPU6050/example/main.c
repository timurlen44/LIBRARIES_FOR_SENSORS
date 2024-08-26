/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MPU6050_LIB.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

double C = 0.00;//Celcius
double F = 0.00;//Fahrenheit
double K = 0.00;//Kelvin

uint16_t GYRO_RAW_X;
uint16_t GYRO_RAW_Y;
uint16_t GYRO_RAW_Z;

uint16_t ACC_RAW_X;
uint16_t ACC_RAW_Y;
uint16_t ACC_RAW_Z;

double GYRO_X;
double GYRO_Y;
double GYRO_Z;

double ACC_X;
double ACC_Y;
double ACC_Z;

double ROLL;
double PITCH;
double YAW;

double ROLL_COMPLEMANTARY_FILTER;
double PITCH_COMPLEMANTARY_FILTER;
double YAW_COMPLEMANTARY_FILTER;

double ROLL_LOW_PASS_FILTER;
double PITCH_LOW_PASS_FILTER;
double YAW_LOW_PASS_FILTER;

double acc_offset_x, acc_offset_y, acc_offset_z;
double gyro_offset_x, gyro_offset_y, gyro_offset_z;
uint16_t pulse = 500;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_TIM3_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  if(MPU6050_INIT(MPU6050_ADDR, 100, FS_SEL_250, AFS_SEL_2g) != MPU6050_OK)
  {
	  while(1);
  }


  /*
  //these are the offset values which i calculated for my mpu6050
  double acc_offset_x = 0.024008789062500001, acc_offset_y = -0.056934814453125003, acc_offset_z = -0.053457031250000009;
  double gyro_offset_x = -4.3777480916030518, gyro_offset_y = -0.53461832061068693, gyro_offset_z = 0.22587786259541978;
  */

  // Calculating The Offset begin
  // !!! Important !!! =====> before calculating the offset values, you should place the device on a flat surface and never move it.
  HAL_Delay(1000);
  //Calculating offset
  MPU6050_Calculate_Offsets(&acc_offset_x, &acc_offset_y, &acc_offset_z, &gyro_offset_x, &gyro_offset_y, &gyro_offset_z);
  MPU6050_Set_Offsets(acc_offset_x, acc_offset_y, acc_offset_z, gyro_offset_x, gyro_offset_y, gyro_offset_z);
  //Calculating The Offsets end


  double MULTIPLE_COEFFICIENT_OF_PREVIOUS_DATA = 0.04;
  double MULTIPLE_COEFFICIENT_OF_PRESENT_DATA = 0.96;
  MPU6050_Set_Low_Pass_Filter_Coefficient(MULTIPLE_COEFFICIENT_OF_PREVIOUS_DATA, MULTIPLE_COEFFICIENT_OF_PRESENT_DATA);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

		  MPU6050_Update_All();// Get raw datas from mpu6050
		  MPU6050_Process_All_Data(); // Processing raw datas

		  //Temprature
		  C = MPU6050_Get_Temperature_In_Celcius(); //Get temprature in celcius
		  F = MPU6050_Get_Temperature_In_Fahrenheit();//Get temprature in Fahrenheit
		  K = MPU6050_Get_Temperature_In_Kelvin();//Get temprature in Kelvin

		  //Raw gyroscope data
		  GYRO_RAW_X = MPU6050_Get_Raw_Gyro_X();
		  GYRO_RAW_Y = MPU6050_Get_Raw_Gyro_Y();
		  GYRO_RAW_Z = MPU6050_Get_Raw_Gyro_Z();

		  //Raw acceleration data
		  ACC_RAW_X = MPU6050_Get_Raw_Acc_X();
		  ACC_RAW_Y = MPU6050_Get_Raw_Acc_Y();
		  ACC_RAW_Z = MPU6050_Get_Raw_Acc_Z();

		  //Gyroscope data
		  GYRO_X = MPU6050_Get_Gyro_X();
		  GYRO_Y = MPU6050_Get_Gyro_Y();
		  GYRO_Z = MPU6050_Get_Gyro_Z();

		  //Gyroscope data
		  ACC_X = MPU6050_Get_Acc_X();
		  ACC_Y = MPU6050_Get_Acc_Y();
		  ACC_Z = MPU6050_Get_Acc_Z();

		  //Euler angles without filter
		  ROLL  = MPU6050_Get_Roll();
		  PITCH = MPU6050_Get_Pitch();
		  YAW   = MPU6050_Get_Yaw();

		  //Euler angles with complemantary filter.
		  ROLL_COMPLEMANTARY_FILTER  = MPU6050_Get_Roll_With_Complemantary_Filter();
		  PITCH_COMPLEMANTARY_FILTER = MPU6050_Get_Pitch_With_Complemantary_Filter();
		  YAW_COMPLEMANTARY_FILTER   = MPU6050_Get_Yaw_With_Complemantary_Filter();

		  //Euler angles with low pass filter.
		  ROLL_LOW_PASS_FILTER  = MPU6050_Get_Roll_With_Low_Pass_Filter();
		  PITCH_LOW_PASS_FILTER = MPU6050_Get_Pitch_With_Low_Pass_Filter();
		  YAW_LOW_PASS_FILTER   = MPU6050_Get_Yaw_With_Low_Pass_Filter();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI9_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
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
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MPU6050_IRQ_Pin_Pin */
  GPIO_InitStruct.Pin = MPU6050_IRQ_Pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MPU6050_IRQ_Pin_GPIO_Port, &GPIO_InitStruct);

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
