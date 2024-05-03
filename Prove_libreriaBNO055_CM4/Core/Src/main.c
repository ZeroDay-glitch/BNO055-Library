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
#include "string.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bno055.h"
#include "stdio.h"
#include "stdint.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x30000000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x30000200
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x30000000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x30000200))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */
ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */

#endif

ETH_TxPacketConfig TxConfig;

ETH_HandleTypeDef heth;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
int flag_BNO055_Data_Ready = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	/* Reset of all peripherals, Initializes the Flash interface and the
	     * Systick. */
	    HAL_Init();

	    /* Configure the system clock */
	    /* Initialize all configured peripherals */
	    MX_GPIO_Init();
	    MX_I2C1_Init();
	    MX_USART3_UART_Init();

	    bno055_assignI2C(&hi2c1);
		bno055_setup();

		HAL_Delay(1000);

  /* USER CODE END 1 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /*HW semaphore Clock enable*/
  __HAL_RCC_HSEM_CLK_ENABLE();
  /* Activate HSEM notification for Cortex-M4*/
  HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));
  /*
  Domain D2 goes to STOP mode (Cortex-M4 in deep-sleep) waiting for Cortex-M7 to
  perform system initialization (system clock config, external memory configuration.. )
  */
  HAL_PWREx_ClearPendingEvent();
  HAL_PWREx_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE, PWR_D2_DOMAIN);
  /* Clear HSEM flag */
  __HAL_HSEM_CLEAR_FLAG(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));

/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  //HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  //MX_GPIO_Init();
  //MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /*bno055_axis_map_t myAxisMap;
  myAxisMap.x = 0;          // Configura la mappatura dell'asse X
  myAxisMap.y = 1;          // Configura la mappatura dell'asse Y
  myAxisMap.z = 2;          // Configura la mappatura dell'asse Z
  myAxisMap.x_sign = 0;     // Configura il segno dell'asse X
  myAxisMap.y_sign = 0;     // Configura il segno dell'asse Y
  myAxisMap.z_sign = 0;     // Configura il segno dell'asse Z

  bno055_setAxisMap(myAxisMap);*/


  /*bno055_setOperationModeConfig();
  bno055_calibration_state_t initialCalState = bno055_getCalibrationState();
     printf("Stato di calibrazione iniziale: Sys: %d, Gyro: %d, Mag: %d, Accel: %d\r\n",
            initialCalState.sys, initialCalState.gyro, initialCalState.mag, initialCalState.accel);*/

  bno055_vector_t gyr;
  bno055_vector_t acc;
  bno055_vector_t mag;

  bno055_setOperationModeAMG();

/*
  bno055_setOperationModeNDOF();
  bno055_vector_xyz_int16_t accelOffset = calibrateAccel();
  setAccelCalibration(accelOffset);

  bno055_calibration_state_t finalCalState = bno055_getCalibrationState();
  printf("Stato di calibrazione finale: Sys: %d, Gyro: %d, Mag: %d, Accel: %d\r\n",
		finalCalState.sys, finalCalState.gyro, finalCalState.mag, finalCalState.accel);*/


  //bno055_setOperationModeCOMPASS();
  //bno055_vector_t quat;
  //bno055_vector_t eul;

/*
  uint8_t gRange = 2;
  float bandwidth = 31.25;
  uint8_t operationMode = NORMAL;

  int gRange2 = 125;
  float bandwidth2 = 32;
  uint8_t operationMode2 = NORMAL;

  uint8_t dataOutputRate  = 2;
  uint8_t operationMode3 = REGULAR;
  uint8_t powerMode = NORMAL;

  //bno055_setOperationModeConfig();

  bno055_configureGyroscope(gRange2, bandwidth2, operationMode2);
  bno055_configureMagnetometer(dataOutputRate, operationMode3, powerMode);
  bno055_configureAccelerometer(gRange, bandwidth, operationMode);*/


  //bno055_setPowerMode(NORMAL_MODE);

  //PowerMode currentMode = bno055_getPowerMode();
  //printf("Modalità energetica attuale: %d\r\n", currentMode);

  //bno055_configureUnits(0, 0 , 0, 0, 0);
  //bno055_setOperationModeNDOF();

  //bno055_setOperationModeConfig();
  //bno055_setOperationModeNDOF();
  //bno055_self_test_result_t res = bno055_getSelfTestResult();

  //printf("MCU - %u | GYR - %u | MAG - %u | ACC - %u\r\n", res.mcuState, res.gyrState, res.magState, res.accState);

  //HAL_Delay(600);
/*
  uint8_t tmp = bno055_getSystemStatus();
  uint8_t err = bno055_getSystemError();
  printf("STATE - %u | ERR - %u\r\n ", tmp, err);

  bno055_setOperationModeConfig();

  uint8_t tmp2 = bno055_getSystemStatus();
  uint8_t err2 = bno055_getSystemError();
  printf("STATE - %u | ERR - %u\r\n ", tmp2, err2);

  HAL_Delay(600);
*/

  //bno055_setInterruptON();
  /*bno055_set_int_en(0, 1, 0, 0, 0, 0, 0, 0);
  bno055_set_int_msk(0, 1, 0, 0, 0, 0, 0, 0);
  bno055_set_acc_int_settings(0, 0, 0, 1, 0, 0, 1, 1);
  bno055_set_gyr_int_settings(1, 1, 1, 1, 1, 1, 1, 1);

  bno055_get_int_en();
  bno055_get_int_msk();
  bno055_get_acc_int_settings();
  bno055_get_gyr_int_settings();
  //bno055_setOperationModeACCONLY();
*/
  //bno055_setOperationModeNDOF();
  //uint8_t datu[1];
  //uint8_t len = 1;
  //bno055_setOperationModeACCONLY();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(flag_BNO055_Data_Ready==1){
	  	  flag_BNO055_Data_Ready = 1;
	  	  //bno055_writeData(BNO055_SYS_TRIGGER, 0x40); //reset int

	  	  //bno055_setOperationModeNDOF();
	  	  //bno055_vector_t acc = bno055_getVectorAccelerometer(); // Legge i dati calibrati
	      //printf("ACC - x: %+2.2f | y: %+2.2f | z: %+2.2f\r\n", acc.x, acc.y, acc.z);

	      //bno055_setOperationModeACCONLY();
	  	  //bno055_calibration_state_t cal = bno055_getCalibrationState();
	      //gyr = bno055_getVectorGyroscope();
		  //acc = bno055_getVectorAccelerometer();
	  	  //mag = bno055_getVectorMagnetometer();
	  	  //quat = bno055_getVectorQuaternion();

	  	  //float heading = calculateHeading(mag.x, mag.y);

	  	  //printf("Orientamento magnetico: %f, cal - acc: %d | mag: %d\r\n", heading, cal.accel, cal.mag);

	  	  //eul = bno055_getVectorEuler();
	  	  //bno055_readData(BNO055_INT_STATUS, datu, len);
	  	  //printf("Dati letti dal registro BNO055_INT_STA: %02X\r\n", datu[0]);

	  	  //printf("GYR - x: %+2.2f | y: %+2.2f | z: %+2.2f | %+2.2d\r\n", gyr.x, gyr.y, gyr.z, cal.gyro);
	  	  //printf("ACC - x: %+2.2f | y: %+2.2f | z: %+2.2f | %+2.2d\r\n", acc.x, acc.y, acc.z, cal.accel);
	  	  //printf("MAG - x: %+2.2f | y: %+2.2f | z: %+2.2f | %+2.2d\r\n", mag.x, mag.y, mag.z, cal.mag);
	  	  //printf("QUAT - x: %+2.2f | y: %+2.2f | z: %+2.2f | w: %+2.2f | %+2.2d\r\n", quat.x, quat.y, quat.z, quat.w, cal.sys);
	  	  //printf("GYR : %+2.2d | ACC : %+2.2d | MAG : %+2.2d | %+2.2d\r\n", cal.gyro, cal.accel, cal.mag, cal.sys);
	  	  //printf("EUL - Yaw: %+2.2f | Roll: %+2.2f | Pitch: %+2.2f | %+2.2d\r\n", eul.x, eul.y, eul.z, cal.sys);

	  	  //bno055_calibration_data_t calData = bno055_getCalibrationData();
	      //bno055_setCalibrationData(calData);

	  	  //printf("\r\n");

	  	 gyr = bno055_getVectorGyroscope();
	  	 acc = bno055_getVectorAccelerometer();
	  	 mag = bno055_getVectorMagnetometer();
	  	 printf("GYR-x: %+2.2f | y: %+2.2f | z: %+2.2f\r\n", gyr.x,
	  	 gyr.y, gyr.z);
	  	 printf("ACC-x: %+2.2f | y: %+2.2f | z: %+2.2f\r\n", acc.x,
	  	 acc.y, acc.z);
	  	 printf("MAG-x: %+2.2f | y: %+2.2f | z: %+2.2f\r\n", mag.x,
	  	 mag.y, mag.z);

	  	  HAL_Delay(100);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  }
  }
  /* USER CODE END 3 */
}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

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
  hi2c1.Init.Timing = 0x00909FCE;
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

int __io_putchar(int ch){
	HAL_UART_Transmit(&huart3, (uint8_t*)&ch, 1, 0xFFFF);
	return ch;
}

int __io_getchar(void)
{
	uint8_t ch;

	__HAL_UART_CLEAR_OREFLAG(&huart3);

	HAL_UART_Receive(&huart3, (uint8_t *)&ch, 1, 0xFFFF);

	HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);

	return ch;
}

void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_3){
		flag_BNO055_Data_Ready = 1;

	}
}

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
