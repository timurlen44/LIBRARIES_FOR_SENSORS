/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "JN2101_LIB.h"
#include "FastKalmanFilter2.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int c = 0;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles USB low priority or CAN RX0 interrupts.
  */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 0 */
	c++;
	if(HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxReceiver, received_data_from_jn2101)== HAL_OK)
	{
		//Is_Message_Received = Message_Is_Received;

		if((received_data_from_jn2101[1] == 0x10) && (received_data_from_jn2101[2] == 0x60) && (received_data_from_jn2101[3] == 0x00))
		{
			jn2101.x_axis_raw = (uint16_t) received_data_from_jn2101[4] + (uint16_t) (received_data_from_jn2101[5] << 8);

			//jn2101.x_axis_degree = round(jn2101.x_axis_raw/100.0);
			jn2101.x_axis_degree = jn2101.x_axis_raw/100.0;
			jn2101.filtered_x_axis_degree = GetEstimation_sensor1((jn2101.x_axis_raw/100.0), 0.0);
		}
		else if((received_data_from_jn2101[1] == 0x20) && (received_data_from_jn2101[2] == 0x60) && (received_data_from_jn2101[3] == 0x00))
		{
			jn2101.y_axis_raw = (uint16_t) received_data_from_jn2101[4] + (uint16_t) (received_data_from_jn2101[5] << 8);
			jn2101.y_axis_degree = jn2101.y_axis_raw/100.0;
			jn2101.filtered_y_axis_degree = GetEstimation_sensor2((jn2101.y_axis_raw/100.0), 0.0);
		}
		else if ( (received_data_from_jn2101[1] == 0x00) && (received_data_from_jn2101[2] == 0x20) && (received_data_from_jn2101[3] == 0x00))
		{
			jn2101.Node_ID = received_data_from_jn2101[4];
		}
		else if ((received_data_from_jn2101[1] == 0x01) && (received_data_from_jn2101[2] == 0x20) && (received_data_from_jn2101[3] == 0x00))
		{
			jn2101.Baud_Rate = received_data_from_jn2101[4] + (received_data_from_jn2101[5]<<8); // Normalde 2 byte uzunluğunda ama bu durumda 2 byte uzunluğunda olmasını göz ardı ediceğim.
		}
		else if((received_data_from_jn2101[1] == 0x18) && (received_data_from_jn2101[2] == 0x10) && (received_data_from_jn2101[3] == 0x00) && (received_data_from_jn2101[4] == 0x04))
		{
			jn2101.Is_Device_Connected = Device_Is_Connected;
		}
		else if((received_data_from_jn2101[1] == 0x18) && (received_data_from_jn2101[2] == 0x10) && (received_data_from_jn2101[3] == 0x01))
		{
			jn2101.Vendor_ID = (uint32_t) received_data_from_jn2101[7] + (uint32_t) (received_data_from_jn2101[6] << 8)+ (uint32_t) (received_data_from_jn2101[5] << 16)+ (uint32_t) (received_data_from_jn2101[4] << 24);
		}
		else if((received_data_from_jn2101[1] == 0x18) && (received_data_from_jn2101[2] == 0x10) && (received_data_from_jn2101[3] == 0x02))
		{
			jn2101.Product_Code = (uint32_t) received_data_from_jn2101[4] + (uint32_t) (received_data_from_jn2101[5] << 8)+ (uint32_t) (received_data_from_jn2101[6] << 16)+ (uint32_t) (received_data_from_jn2101[7] << 24);
		}
		else if((received_data_from_jn2101[1] == 0x18) && (received_data_from_jn2101[2] == 0x10) && (received_data_from_jn2101[3] == 0x03))
		{
			jn2101.Revision_Number = (uint32_t) received_data_from_jn2101[4] + (uint32_t) (received_data_from_jn2101[5] << 8)+ (uint32_t) (received_data_from_jn2101[6] << 16)+ (uint32_t) (received_data_from_jn2101[7] << 24);
		}
		else if((received_data_from_jn2101[1] == 0x18) && (received_data_from_jn2101[2] == 0x10) && (received_data_from_jn2101[3] == 0x04))
		{
			jn2101.Serial_Number = (uint32_t) received_data_from_jn2101[7] + (uint32_t) (received_data_from_jn2101[6] << 8)+ (uint32_t) (received_data_from_jn2101[5] << 16)+ (uint32_t) (received_data_from_jn2101[4] << 24);
		}

		else if((received_data_from_jn2101[1] == 0x43) && (received_data_from_jn2101[2] == 0x20) && (received_data_from_jn2101[3] == 0x00))
		{
			jn2101.FIR_Filter_Hz = received_data_from_jn2101[4];
		}

		//memset((char*)received_data_from_jn2101,'\0',8);
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	}
  /* USER CODE END USB_LP_CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan);
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 1 */

  /* USER CODE END USB_LP_CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(B1_Pin);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
