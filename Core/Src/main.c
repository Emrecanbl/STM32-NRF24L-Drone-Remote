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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c-lcd.h"
#include "ssd1306.h"
#include "nRF24L01P.h"
#include "dwt_delay.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
void NRFSendData(char * nrfID, char * nrfData);
void NRF_main_init(void);


uint32_t adc_values[4];
nRF24L01P myNRF;
char RXBuffer[32];
char TXBuffer[32];
char Adc_Val1[24];
char Adc_Val2[32];
char regStatus;
JoyStick_Val pJoy;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  ssd1306_Init();
  HAL_Delay(100);
  DWT_Init();
  NRF_main_init();
  HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.Timing = 0x10707DBC;
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, NRF24L_CE_Pin|NRF24L_CSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : NRF24L_interrupt_Pin */
  GPIO_InitStruct.Pin = NRF24L_interrupt_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(NRF24L_interrupt_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : NRF24L_CE_Pin NRF24L_CSN_Pin */
  GPIO_InitStruct.Pin = NRF24L_CE_Pin|NRF24L_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Buton_1_Pin */
  GPIO_InitStruct.Pin = Buton_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Buton_1_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_2)
	{
		if(HAL_nRF24L01P_IRQ_Handler(&myNRF) != HAL_OK)
		{
			Error_Handler();
		}
		HAL_nRF24L01P_ReadRegister(&myNRF, nRF_STATUS, &regStatus);
	}
}
void NRFSendData(char * nrfID, char * nrfData)
{
	HAL_nRF24L01P_SetPTXAddress(&myNRF, (uint8_t *) nrfID);
	if(HAL_nRF24L01P_TransmitPacketACK(&myNRF, (uint8_t *) nrfData, nRF_DATA_PIPE_0) != HAL_OK)
	{
		//Error
	}
}

void NRF_main_init(void)
{
	/* ---- myNRF24L01+ Definitions ---- */
	myNRF.hspi = &hspi1;
	myNRF.CRC_Width = nRF_CRC_WIDTH_BYTE;
	myNRF.ADDR_Width = nRF_ADDR_WIDTH_5;
	myNRF.Data_Rate = nRF_DATA_RATE_2MBPS;
	myNRF.TX_Power = nRF_TX_PWR_M18dBm;
	myNRF.State = nRF_STATE_RX;

	myNRF.RF_Channel = 1;
	myNRF.PayloadWidth = nRF_RXPW_32BYTES;
	myNRF.RetransmitCount = nRF_RETX_DISABLED;// nRF_RETX_COUNT_15;
	myNRF.RetransmitDelay = nRF_RETX_DELAY_250uS; //nRF_RETX_DELAY_4000uS;

	myNRF.RX_Address = (uint8_t *)"00001";
	myNRF.TX_Address = (uint8_t *)"00000";

	myNRF.RX_Buffer = RXBuffer;
	myNRF.TX_Buffer = TXBuffer;

	myNRF.nRF_nSS_GPIO_PORT = GPIOB;
	myNRF.nRF_nSS_GPIO_PIN = GPIO_PIN_1;

	myNRF.nRF_CE_GPIO_PORT = GPIOB;
	myNRF.nRF_CE_GPIO_PIN = GPIO_PIN_0;

	//ekleme
	myNRF.RXIRQ = 0;
	myNRF.TXIRQ = 0;
	myNRF.MaxReIRQ = 0;
	/* ---- myNRF24L01+ Definitions ---- */


	if(HAL_nRF24L01P_Init(&myNRF) != HAL_OK)
	{
		Error_Handler();
	}

}
void Lcd_update(uint8_t vol){

	ssd1306_SetCursor(12,0);
	ssd1306_WriteString("Battery Voltage", Font_7x10, White);
	char str[4];
	ssd1306_SetCursor(45,12);
	uint8_t vol1=vol/10;
	uint8_t vol2=vol%10;
	sprintf(str, "%d", vol1);
	ssd1306_WriteString(str, Font_11x18, White);
	ssd1306_SetCursor(57,12);
	ssd1306_WriteString(".", Font_11x18, White);
	sprintf(str, "%d", vol2);
	ssd1306_SetCursor(68,12);
	ssd1306_WriteString(str, Font_11x18, White);
	ssd1306_UpdateScreen();
}
void Joystick_read(JoyStick_Val *pJoy){
	HAL_ADC_Start_DMA(&hadc1, adc_values, 4); //ADC Read for JOysticks
	uint16_t JoyStick_Throttle_Val = adc_values[1];

	if(JoyStick_Throttle_Val >=2100){
		JoyStick_Throttle_Val = (JoyStick_Throttle_Val - 2100)/8;		//For 8 Bit Value Trasfer
		pJoy->Trottle_pos_Up = JoyStick_Throttle_Val;
		pJoy->Trottle_pos_Down=0;
	}
	else if (JoyStick_Throttle_Val <=1900){
		JoyStick_Throttle_Val = ( 1900 - JoyStick_Throttle_Val)/8;
		pJoy->Trottle_pos_Down = JoyStick_Throttle_Val;
		pJoy->Trottle_pos_Up =0;
	}
	else{
		pJoy->Trottle_pos_Up=0;
		pJoy->Trottle_pos_Down=0;
	}

	uint16_t JoyStick_Yaw_Val = adc_values[0];

	if(JoyStick_Yaw_Val >=2100){
		JoyStick_Yaw_Val = (JoyStick_Yaw_Val - 2100)/8;		//For 8 Bit Value Trasfer
		pJoy->Yaw_pos_Right = JoyStick_Yaw_Val;
		pJoy->Yaw_pos_Left = 0;
		}
	else if (JoyStick_Yaw_Val <=1900){
		JoyStick_Yaw_Val = ( 1900 - JoyStick_Yaw_Val)/8;
		pJoy->Yaw_pos_Left = JoyStick_Yaw_Val;
		pJoy->Yaw_pos_Right = 0;
		}
	else{
			pJoy->Yaw_pos_Right=0;
			pJoy->Yaw_pos_Left=0;
		}
	uint16_t JoyStick_Roll_Val = adc_values[3];

	if(JoyStick_Roll_Val >=2100){
		JoyStick_Roll_Val = (JoyStick_Roll_Val - 2100)/8;		//For 8 Bit Value Trasfer
		pJoy->Roll_pos_Left = JoyStick_Roll_Val;
		pJoy->Roll_pos_Right = 0;
		}
	else if (JoyStick_Roll_Val <=1900){
		JoyStick_Roll_Val = ( 1900 - JoyStick_Roll_Val)/8;
		pJoy->Roll_pos_Right = JoyStick_Roll_Val;
		pJoy->Roll_pos_Left = 0;
		}
	else{
			pJoy->Roll_pos_Right=0;
			pJoy->Roll_pos_Left=0;
		}
	uint16_t JoyStick_Pitch_Val = adc_values[2];

	if(JoyStick_Pitch_Val >=2100){
		JoyStick_Pitch_Val = (JoyStick_Pitch_Val - 2100)/8;		//For 8 Bit Value Trasfer
		pJoy->Pitch_pos_Up = JoyStick_Pitch_Val;
		pJoy->Pitch_pos_Down = 0;
		}
	else if (JoyStick_Pitch_Val <=1900){
		JoyStick_Pitch_Val = ( 1900 - JoyStick_Pitch_Val)/8;
		pJoy->Pitch_pos_Down = JoyStick_Pitch_Val;
		pJoy->Pitch_pos_Up = 0;
		}
	else{
			pJoy->Pitch_pos_Up=0;
			pJoy->Pitch_pos_Down=0;
		}

	uint8_t ADC11=pJoy->Trottle_pos_Up;
	uint8_t ADC2=pJoy->Trottle_pos_Down;
	uint8_t ADC3=pJoy->Yaw_pos_Right;
	uint8_t ADC4=pJoy->Yaw_pos_Left;
	uint8_t ADC5=pJoy->Roll_pos_Left;
	uint8_t ADC6=pJoy->Roll_pos_Right;
	uint8_t ADC7=pJoy->Pitch_pos_Up;
	uint8_t ADC8=pJoy->Pitch_pos_Down;

	sprintf(Adc_Val1, "%0.3i%0.3i%0.3i%0.3i%0.3i%0.3i%0.3i%0.3i", ADC11,ADC2,ADC3,ADC4, ADC5,ADC6,ADC7,ADC8);
	HAL_Delay(1);
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
