/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Fusion.h"
#include <stdio.h>
#include "sensor.h"
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
I2C_HandleTypeDef hi2c2;

LCD_HandleTypeDef hlcd;

QSPI_HandleTypeDef hqspi;

SAI_HandleTypeDef hsai_BlockA1;
SAI_HandleTypeDef hsai_BlockB1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
packet_with_control data_to_send;	// packet with control to PC

float control_value;// Control value in range[-1 ... 1]
float control_value2;//

// board sensor raw variables
float gyro_data[3] = {0.0};
float acc_data[3] = {0.0};
float mag_data[3] = {0.0};

//UART buffer
char UARTbuffer[64];
size_t buffer_size;

//Loop period
float deltaTime = 50.0/1000.0;

FusionAhrs ahrs;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_LCD_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_SAI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM7_Init(void);
void MX_USB_HOST_Process(void);

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
	// copy PC uart pointer to external variable
	uart_to_pc_ptr = (UART_HandleTypeDef*) &huart2;
	// init control value to 0
	control_value = 0;
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
  MX_I2C2_Init();
  MX_LCD_Init();
  MX_QUADSPI_Init();
  MX_SAI1_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_USB_HOST_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  // Init on board sensors
  BSP_GYRO_Init();
  BSP_COMPASS_Init();

  // AHRS Madgwick
  FusionOffset offset;
//  FusionOffsetInitialise(&offset, 20);
  FusionAhrsInitialise(&ahrs);

  const FusionAhrsSettings settings = {
          .gain = 0.5f,
          .accelerationRejection = 10.0f,
          .magneticRejection = 20.0f,
          .rejectionTimeout = 5,//5 * 20, /* 5 seconds */
  };
  FusionAhrsSetSettings(&ahrs, &settings);

  HAL_TIM_Base_Start_IT(&htim7);
  // select initial control value
//  data_to_send.control = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */
    // sleep for 50ms
    //TODO: change this delay to more real time
    HAL_Delay(50);  // [ms]
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_SAI1|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_I2C2|RCC_PERIPHCLK_USB;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  PeriphClkInit.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK|RCC_PLLSAI1_48M2CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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
  hi2c1.Init.Timing = 0x10909CEC;
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10909CEC;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief LCD Initialization Function
  * @param None
  * @retval None
  */
static void MX_LCD_Init(void)
{

  /* USER CODE BEGIN LCD_Init 0 */

  /* USER CODE END LCD_Init 0 */

  /* USER CODE BEGIN LCD_Init 1 */

  /* USER CODE END LCD_Init 1 */
  hlcd.Instance = LCD;
  hlcd.Init.Prescaler = LCD_PRESCALER_1;
  hlcd.Init.Divider = LCD_DIVIDER_16;
  hlcd.Init.Duty = LCD_DUTY_1_4;
  hlcd.Init.Bias = LCD_BIAS_1_4;
  hlcd.Init.VoltageSource = LCD_VOLTAGESOURCE_INTERNAL;
  hlcd.Init.Contrast = LCD_CONTRASTLEVEL_0;
  hlcd.Init.DeadTime = LCD_DEADTIME_0;
  hlcd.Init.PulseOnDuration = LCD_PULSEONDURATION_0;
  hlcd.Init.MuxSegment = LCD_MUXSEGMENT_DISABLE;
  hlcd.Init.BlinkMode = LCD_BLINKMODE_OFF;
  hlcd.Init.BlinkFrequency = LCD_BLINKFREQUENCY_DIV8;
  hlcd.Init.HighDrive = LCD_HIGHDRIVE_DISABLE;
  if (HAL_LCD_Init(&hlcd) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LCD_Init 2 */

  /* USER CODE END LCD_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 255;
  hqspi.Init.FifoThreshold = 1;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
  hqspi.Init.FlashSize = 1;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief SAI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAI1_Init(void)
{

  /* USER CODE BEGIN SAI1_Init 0 */

  /* USER CODE END SAI1_Init 0 */

  /* USER CODE BEGIN SAI1_Init 1 */

  /* USER CODE END SAI1_Init 1 */
  hsai_BlockA1.Instance = SAI1_Block_A;
  hsai_BlockA1.Init.Protocol = SAI_FREE_PROTOCOL;
  hsai_BlockA1.Init.AudioMode = SAI_MODEMASTER_TX;
  hsai_BlockA1.Init.DataSize = SAI_DATASIZE_8;
  hsai_BlockA1.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockA1.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
  hsai_BlockA1.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA1.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_192K;
  hsai_BlockA1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  hsai_BlockA1.FrameInit.FrameLength = 8;
  hsai_BlockA1.FrameInit.ActiveFrameLength = 1;
  hsai_BlockA1.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
  hsai_BlockA1.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  hsai_BlockA1.FrameInit.FSOffset = SAI_FS_FIRSTBIT;
  hsai_BlockA1.SlotInit.FirstBitOffset = 0;
  hsai_BlockA1.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
  hsai_BlockA1.SlotInit.SlotNumber = 1;
  hsai_BlockA1.SlotInit.SlotActive = 0x00000000;
  if (HAL_SAI_Init(&hsai_BlockA1) != HAL_OK)
  {
    Error_Handler();
  }
  hsai_BlockB1.Instance = SAI1_Block_B;
  hsai_BlockB1.Init.Protocol = SAI_FREE_PROTOCOL;
  hsai_BlockB1.Init.AudioMode = SAI_MODESLAVE_RX;
  hsai_BlockB1.Init.DataSize = SAI_DATASIZE_8;
  hsai_BlockB1.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockB1.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
  hsai_BlockB1.Init.Synchro = SAI_SYNCHRONOUS;
  hsai_BlockB1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockB1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockB1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockB1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockB1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockB1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  hsai_BlockB1.FrameInit.FrameLength = 8;
  hsai_BlockB1.FrameInit.ActiveFrameLength = 1;
  hsai_BlockB1.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
  hsai_BlockB1.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  hsai_BlockB1.FrameInit.FSOffset = SAI_FS_FIRSTBIT;
  hsai_BlockB1.SlotInit.FirstBitOffset = 0;
  hsai_BlockB1.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
  hsai_BlockB1.SlotInit.SlotNumber = 1;
  hsai_BlockB1.SlotInit.SlotActive = 0x00000000;
  if (HAL_SAI_Init(&hsai_BlockB1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI1_Init 2 */

  /* USER CODE END SAI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 7999;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 499;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  huart2.Init.BaudRate = 115200;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, AUDIO_RST_Pin|LD_G_Pin|XL_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD_R_Pin|M3V3_REG_ON_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_VBUS_GPIO_Port, OTG_FS_VBUS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : AUDIO_RST_Pin */
  GPIO_InitStruct.Pin = AUDIO_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(AUDIO_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MFX_IRQ_OUT_Pin OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = MFX_IRQ_OUT_Pin|OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 MAG_INT_Pin MAG_DRDY_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|MAG_INT_Pin|MAG_DRDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : JOY_CENTER_Pin JOY_LEFT_Pin JOY_RIGHT_Pin JOY_UP_Pin
                           JOY_DOWN_Pin */
  GPIO_InitStruct.Pin = JOY_CENTER_Pin|JOY_LEFT_Pin|JOY_RIGHT_Pin|JOY_UP_Pin
                          |JOY_DOWN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : MFX_WAKEUP_Pin */
  GPIO_InitStruct.Pin = MFX_WAKEUP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MFX_WAKEUP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD_R_Pin */
  GPIO_InitStruct.Pin = LD_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(LD_R_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD_G_Pin */
  GPIO_InitStruct.Pin = LD_G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(LD_G_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_PowerSwitchOn_Pin OTG_FS_VBUS_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin|OTG_FS_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : EXT_RST_Pin GYRO_INT1_Pin */
  GPIO_InitStruct.Pin = EXT_RST_Pin|GYRO_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : GYRO_CS_Pin */
  GPIO_InitStruct.Pin = GYRO_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GYRO_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : M3V3_REG_ON_Pin */
  GPIO_InitStruct.Pin = M3V3_REG_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(M3V3_REG_ON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GYRO_INT2_Pin */
  GPIO_InitStruct.Pin = GYRO_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GYRO_INT2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : XL_CS_Pin */
  GPIO_InitStruct.Pin = XL_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(XL_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : XL_INT_Pin */
  GPIO_InitStruct.Pin = XL_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(XL_INT_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/*
 * Madgwick quaterion to yaw-pitch-roll angles
 */
void compute_yaw_pitch_roll(FusionQuaternion _quat, float * ypt_data){
	   	float x = _quat.element.x;
	   	float y = _quat.element.y;
	   	float z = _quat.element.z;
	   	float w = _quat.element.w;

	   	float yaw = atan2(2.0*(y*z + w*x), w*w - x*x - y*y + z*z);
	   	float pitch = asin(-2.0*(x*z - w*y));
	   	float roll = atan2(2.0*(x*y + w*z), w*w + x*x - y*y - z*z);

	   	ypt_data[0] = yaw;
	   	ypt_data[1] = pitch;
	   	ypt_data[2] = roll;

	    return ;
}


/*
 * Main control loop
 * Call this function for each 50ms
 */
void control_function(){
	//	  control_value = read_joystick();	// read data from mounted joystick
	//	  control_value2 = read_joystick2();

		  readCompassData(mag_data);
		  readGyroscopeData(gyro_data);
		  readAccelometerData(acc_data);

		  FusionVector gyroscope     = {gyro_data[0], gyro_data[1], gyro_data[2]}; // replace this with actual gyroscope data in degrees/s
		  FusionVector accelerometer = {acc_data[0], acc_data[1], acc_data[2]}; // replace this with actual accelerometer data in g
		  FusionVector magnetometer  = {mag_data[0], mag_data[1], mag_data[2]};

		  FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, deltaTime);
//		  FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, deltaTime);

		  const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
//		  float angles [3];
//		  compute_yaw_pitch_roll(FusionAhrsGetQuaternion(&ahrs), angles);

	//	  const FusionVector earth = FusionAhrsGetEarthAcceleration(&ahrs);

	//	  buffer_size = sprintf(UARTbuffer, "Roll %0.1f, Pitch %0.1f, Yaw %0.1f \r\n",
	//	  			  euler.angle.roll, euler.angle.pitch, euler.angle.yaw);

	//	  buffer_size = sprintf(UARTbuffer, "Roll %0.1f, Pitch %0.1f, Yaw %0.1f, X %0.1f, Y %0.1f, Z %0.1f \r\n",
	//			  euler.angle.roll, euler.angle.pitch, euler.angle.yaw,
	//			  earth.axis.x, earth.axis.y, earth.axis.z);

//		  HAL_UART_Transmit(&huart2, UARTbuffer, buffer_size, HAL_MAX_DELAY);

		  //send control to PC
		  //		  data_to_send.control  = angles[1];
		  //		  data_to_send.control2 = angles[2];
		  data_to_send.control  = euler.angle.roll/180;	// set control value to packet
		  data_to_send.control2 = euler.angle.pitch/180;

		  // delete offset in angle in special position
//		  float roll_offset_reset = 0.8;
//		  if(fabs(data_to_send.control) > roll_offset_reset)
//			  data_to_send.control -= copysignf(roll_offset_reset, data_to_send.control);
//
//		  send_control_packet(data_to_send);	// send new packet to PC

		  buffer_size = sprintf(UARTbuffer, "ACC %0.1f %0.1f %0.1f\t GYR %0.1f %0.1f %0.1f\t MAG %0.1f %0.1f %0.1f \r\n",
				acc_data[0], acc_data[1], acc_data[2],
				gyro_data[0], gyro_data[1], gyro_data[2],
				mag_data[0], mag_data[1], mag_data[2]);

		  HAL_UART_Transmit(&huart2, UARTbuffer, buffer_size, HAL_MAX_DELAY);
}


/*
 * Read control signal from on-board joystick
 */
float read_joystick(){
	// read right and left joystick position
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1))
		return -1;	// return -1 if joystick is in left position
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2))
		return 1;	// return 1 if joystick is in right position

	return 0;	// return 1 if joystick is in right position
}

float read_joystick2(){
	// read right and left joystick position
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3))
		return -1;	// return -1 if joystick is in up position
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5))
		return 1;	// return 1 if joystick is in down position

	return 0;	// return 1 if joystick is in right position
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
