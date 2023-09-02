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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MSG_TYPE_A 0
#define MSG_TYPE_B 1
#define DEVICE_ID 1
#define PASSCODE_LO 0
#define PASSCODE_HI 0 //later both to be changed for AES crypto
#define MSG_SIZE 16
#define STARTER_BUTTON_TIMER 200
#define MSG_A_PERIOD_S 10

#define AT_MODEM_INFO "ATI\n\r"
#define AT_MODEM_STATUS "AT+CFUN?\n\r"
#define AT_MODEM_OFF "AT+CFUN=0\n\r"
#define AT_MODEM_ON "AT+CFUN=1\n\r"
#define AT_SIM_STATUS "AT+CPIN?\n\r"
#define AT_NETLIGHT_ON "AT+QLEDMODE=1\n\r"
#define AT_OPS_SEARCH "AT+COPS=?\n\r"
#define AT_GET_IMSI "AT+CIMI\n\r"
#define AT_NET_STATUS "AT+QENGINFO=0\n\r"
#define AT_FREQ_ULOCK "AT+QFRCLLCK=0\n\r"
#define AT_GET_BAND "AT+QBAND?\n\r"

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t modemRxData[200];
uint8_t rxMsgDataTest[MSG_SIZE];
int cutoffSecCounter;
int cutoffMinutesSet;
uint8_t motorTimerExpired;
uint8_t msgATimerExpired;
int msgASecCounter;

enum GPIO_Pins_Used
{
  NC, NO, VAL0, VAL1
};

void
Start_Modem (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint16_t waitTime,
	     uint16_t enableTime)
{
  HAL_Delay (waitTime);
  HAL_GPIO_WritePin (GPIOx, GPIO_Pin, GPIO_PIN_RESET);
  HAL_Delay (enableTime);
  HAL_GPIO_WritePin (GPIOx, GPIO_Pin, GPIO_PIN_SET);
}

void
Modem_Send_Command (UART_HandleTypeDef *huart, char *cmd)
{
  HAL_UART_Transmit (huart, cmd, strlen (cmd), 10000);
  HAL_Delay (1000);
}

void
Modem_Reset (void)
{
  HAL_GPIO_WritePin (GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
  HAL_Delay (500);
  HAL_GPIO_WritePin (GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
}

void
HAL_UART_RxCpltCallback (UART_HandleTypeDef *huart)
{
  if (huart == &huart2)
    HAL_UART_Receive_IT (&huart2, modemRxData, sizeof(modemRxData));
  else if (huart == &huart1) {
    Handle_Msg_B(rxMsgDataTest);
    memset(rxMsgDataTest, 0, sizeof(rxMsgDataTest));
    HAL_UART_Receive_IT (&huart1, rxMsgDataTest, sizeof(rxMsgDataTest));
  }
}

void
HAL_UART_TxHalfCpltCallback (UART_HandleTypeDef *huart)
{

}

void
HAL_SPI_RxCpltCallback (SPI_HandleTypeDef *hspi)
{

}

uint16_t
ADC_Read (uint8_t channel)
{
  uint8_t adcTx[3] =
    { 0x01, 0x80, 0x00 }; // start bit, channel id
  adcTx[1] = 0x80 | ((channel & 0x03) << 4);
  uint8_t adcRx[3] =
    { 0 };
  HAL_GPIO_WritePin (GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive (&hspi1, adcTx, adcRx, sizeof(adcRx), 100);
  while (hspi1.State == HAL_SPI_STATE_BUSY)
    ;
  HAL_GPIO_WritePin (GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

  uint16_t *retValP = adcRx;
  //uint8_t adcStr[10] = {0};
  //sprintf(adcStr, "%d %d %d\n\r", adcRx[0], adcRx[1], adcRx[2]);
  //HAL_UART_Transmit(&huart1, adcStr, sizeof(adcStr), 1000);
  //uint8_t infoTxt2[] = "\n\r";
  //HAL_UART_Transmit(&huart1, infoTxt2, sizeof(infoTxt2), 1000);

  return *retValP;
}

int8_t
Get_Rssi ()
{
  return 30;
}

void
HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim)
{
  HAL_GPIO_TogglePin (GPIOB, GPIO_PIN_1);
  msgASecCounter += 1;
  if (msgASecCounter == MSG_A_PERIOD_S) {
    msgATimerExpired = 1;
    msgASecCounter = 0;
  }

  if (cutoffMinutesSet > 0) {
    cutoffSecCounter += 1;
  }

  motorTimerExpired = ((cutoffSecCounter / 60) == cutoffMinutesSet);

}

void Start_Motor()
{
  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
  HAL_Delay (STARTER_BUTTON_TIMER);
  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
}

void Stop_Motor()
{
  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
  HAL_Delay (STARTER_BUTTON_TIMER);
  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
}

uint16_t
Get_Rem_Time ()
{
  return (cutoffSecCounter/60);
}

GPIO_PinState
Get_GPIO_State (enum GPIO_Pins_Used pin)
{
  GPIO_PinState state = 0;
  switch (pin)
    {
    case NC:
      state = HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_14);
      break;

    case NO:
      state = HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_15);
      break;

    case VAL0:
      state = HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_2);
      break;

    case VAL1:
      state = HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_8);
      break;
    }
  return state;
}

uint8_t
Gen_GPIO_State_Byte ()
{
  GPIO_PinState ncState = Get_GPIO_State (NC);
  GPIO_PinState noState = Get_GPIO_State (NO);
  GPIO_PinState val0State = Get_GPIO_State (VAL0);
  GPIO_PinState val1State = Get_GPIO_State (VAL1);

  uint8_t byte =
      ((val1State << 3) | (val0State << 2) | (ncState << 1) | noState) & 0xF;

  return byte;
}

void
Gen_Msg_A (uint8_t buffer[MSG_SIZE])
{
  buffer[0] = MSG_TYPE_A;
  buffer[1] = PASSCODE_LO;
  buffer[2] = PASSCODE_HI;
  buffer[3] = DEVICE_ID;
  buffer[4] = Get_Rssi ();

  uint16_t adc0 = ADC_Read (0);
  uint16_t adc1 = ADC_Read (1);
  uint16_t adc2 = ADC_Read (2);
  uint16_t adc3 = ADC_Read (3);
  buffer[5] = adc0 & 0xFF; // 8 bits
  buffer[6] = (adc0 >> 8) & 0x3; // LSB 2 bits

  buffer[6] |= (adc1 << 2) & 0xFC; // MSB 6 bits
  buffer[7] = (adc1 >> 6) & 0xF; // LSB 4 bits

  buffer[7] |= (adc2 << 4) & 0xF0; // MSB 4 bits
  buffer[8] = (adc2 >> 4) & 0x3F0; // LSB 6 bits

  buffer[8] |= (adc3 << 6) & 0xC0; // LSB 2 bits
  buffer[9] = (adc3 >> 2) & 0xFF; // MSB 8 bits

  uint16_t remTime = Get_Rem_Time ();
  buffer[10] = remTime & 0xFF;
  buffer[11] = (remTime >> 8) & 0xFF;

  buffer[12] = Gen_GPIO_State_Byte ();
}

void
Handle_Msg_B (uint8_t buffer[MSG_SIZE])
{
  if (buffer[0] != MSG_TYPE_B)
    return;

  if ((buffer[1] != PASSCODE_LO) || (buffer[2] != PASSCODE_HI))
    return;

  if (buffer[3] != DEVICE_ID)
    return;

  int16_t remTime = ((buffer[5] << 8) & 0xFF00) | (buffer[4] & 0xFF);
  uint8_t motorState = buffer[6] & 0x1;
  uint8_t val0State = (buffer[6] >> 1) & 0x1;
  uint8_t val1State = (buffer[6] >> 2) & 0x1;

  HAL_GPIO_WritePin (GPIOB, GPIO_PIN_2, val0State);
  HAL_GPIO_WritePin (GPIOA, GPIO_PIN_8, val1State);

  cutoffMinutesSet = remTime;
  if (remTime > 0)
    {
      Start_Motor();
    }
  else if (remTime == 0)
    {
      Stop_Motor();
    }
}

void
Print_Modem_Output ()
{
  HAL_UART_Transmit (&huart1, modemRxData, sizeof(modemRxData), 1000);
  HAL_UART_Transmit (&huart1, "\n", 2, 1000);
  memset (modemRxData, 0, sizeof(modemRxData));
}

void Send_Msg_A()
{
  uint8_t buffer[MSG_SIZE];
  memset(buffer, 0, sizeof(buffer));
  Gen_Msg_A(buffer);

  HAL_UART_Transmit (&huart1, buffer, sizeof(buffer), 100);
}
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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
    {
      unsigned int *optionByte = (unsigned int*) 0x1FFF7800;
      const unsigned int boot1Bit = 0x02000000;
      const unsigned int nBootSel = 0xFEFFFFFF;
      *optionByte = ((*optionByte) | boot1Bit) & nBootSel;
    }

  memset (modemRxData, 0, sizeof(modemRxData));
  HAL_UART_Receive_IT (&huart2, modemRxData, sizeof(modemRxData));
  HAL_UART_Receive_IT (&huart1, rxMsgDataTest, sizeof(rxMsgDataTest));

  // ADC CS Pin
  HAL_GPIO_WritePin (GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
  HAL_GPIO_WritePin (GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  Start_Modem (GPIOA, GPIO_PIN_4, 1000, 1000);
  Print_Modem_Output ();
  //Modem_Reset();

  HAL_Delay (1000);
  Modem_Send_Command (&huart2, AT_MODEM_INFO);
  Print_Modem_Output ();

  HAL_Delay (1000);
  Modem_Send_Command (&huart2, AT_MODEM_ON);
  Print_Modem_Output ();

  HAL_Delay (1000);
  Modem_Send_Command (&huart2, AT_NETLIGHT_ON);
  Print_Modem_Output ();

  HAL_Delay (1000);
  Modem_Send_Command (&huart2, AT_SIM_STATUS);
  Print_Modem_Output ();

  HAL_Delay (1000);
  Modem_Send_Command (&huart2, AT_GET_IMSI);
  Print_Modem_Output ();

  HAL_Delay (1000);
  Modem_Send_Command (&huart2, AT_GET_BAND);
  Print_Modem_Output ();

  //HAL_Delay(1000);
  //Modem_Send_Command(&huart2, AT_OPS_SEARCH);
  //HAL_Delay(630000);
  //Print_Modem_Output();

  //SPI test
  //ADC_Read(0);
  //ADC_Read(1);
  //ADC_Read(2);
  //ADC_Read(3);
  msgATimerExpired = 0;
  motorTimerExpired = 0;
  cutoffSecCounter = 0;
  cutoffMinutesSet = -1;
  HAL_TIM_Base_Start_IT (&htim14);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      if (msgATimerExpired) {
	  Send_Msg_A ();
	  msgASecCounter = 0;
	  msgATimerExpired = 0;
      }

      if (motorTimerExpired) {
          Stop_Motor();
          cutoffSecCounter = 0;
          cutoffMinutesSet = -1;
          motorTimerExpired = 0;
        }
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 16000-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 1000-1;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_EVEN;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);

  /*Configure GPIO pins : PB9 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
  __disable_irq ();
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
