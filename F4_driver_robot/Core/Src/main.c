/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define Tran_P 1
#define Tran_V 0
#define Block 2
#define Control_V 1
#define Control_P 0
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
double Kp1 = 14 , Kp2 = 15, Ki1 = 200, Ki2= 200, Kd1 =0, Kd2 =0, Ts =0.005 ; /*PID parameter for  velocity Control*/
double Kp_1 = 15 , Kp_2 = 19, Ki_1 = 60, Ki_2= 50, Kd_1 =2, Kd_2 =3; /*PID parameter for positon Control */
double e1,e2; 
volatile int32_t	encoder_cnt1 = 0, encoder_cnt_pre1 = 0, encoder_cnt2 = 0, encoder_cnt_pre2 = 0,test1,test2, s_pos1 = 0, s_pos2 =0;
volatile short rate1,pulse1,rate2,pulse2,s_rate1 = 0,s_rate2 =0,test, x = 999;
volatile int pid1,pid2;
uint8_t u8_Rx_Data[256],u8_Tx_Data[30] ,u8_Rx_SPI[1], u8_Rx_c, control_mode = Control_V, send_mode = Block;
uint8_t start_buff = 0;
uint8_t end_buff = 255;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
DMA_HandleTypeDef hdma_tim2_up_ch3;
DMA_HandleTypeDef hdma_tim5_up;

UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM1_Init(void);
static void MX_UART4_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int muoi_mu(int a);
int Int2Str(int a, int offset);
void tran_mess( uint8_t mode);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi);
int Str2Int(uint8_t* p , uint8_t i);
uint8_t Pop_Rx_Data();
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
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM1_Init();
  MX_UART4_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_1|TIM_CHANNEL_2); 
	HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_1|TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_SPI_Receive_DMA(&hspi1,u8_Rx_SPI,1);
	HAL_UART_Receive_DMA(&huart4,&u8_Rx_c,1);
	
	
	//
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		/* Process UART*/
		uint8_t c = Pop_Rx_Data();
		uint8_t s_buff1[4],s_buff2[4];
		int i_buff =0;
		if (c == '!')
		{
			c = Pop_Rx_Data();
			switch (c)
			{
				case 'V' :
					c = Pop_Rx_Data();
					while (c != ';' && c !='!')
					{
						s_buff1[i_buff++] = c; 
						c = Pop_Rx_Data(); 
					}
					s_rate1 = Str2Int (s_buff1,i_buff);
					i_buff =0;
					c = Pop_Rx_Data(); 
					while (c != '#' && c !='!')
					{
						s_buff2[i_buff++] = c ;
						c = Pop_Rx_Data(); 
					}
					s_rate2 = Str2Int (s_buff2,i_buff);
					i_buff =0;
					control_mode = Control_V ;
					break;
				case 'P' :
					c = Pop_Rx_Data();
					while (c != ';' && c !='!')
					{
						s_buff1[i_buff++] = c; 
						c = Pop_Rx_Data(); 
					}
					s_pos1 = Str2Int (s_buff1,i_buff);
					i_buff =0;
					c = Pop_Rx_Data(); 
					while (c != '#' && c !='!')
					{
						s_buff2[i_buff++] = c ;
						c = Pop_Rx_Data(); 
					}
					s_pos2 = Str2Int (s_buff2,i_buff);
					i_buff =0;
					control_mode = Control_P ;
					break;
				case 'v':
					c = Pop_Rx_Data();
					if (c == '#') send_mode = Tran_V;
					break;
				case 'p':
					c = Pop_Rx_Data();
					if (c == '#') send_mode = Tran_P;
					break ;
				case 'O':
					c = Pop_Rx_Data();
					if (c == '#') send_mode = Block ;
					break;
				default :
					break;
			}
		}
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
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
  htim1.Init.Prescaler = 499;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1679;
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
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */
	
  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 83;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 999;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
int muoi_mu(int a) /*a >= 0*/
{
	int  r =1;
	if (a == 0){
	return r;
	}
	else if (a < 0){
	return 0;
	}
	r =10*muoi_mu(a -1);
	return r;
}
int Int2Str(int a, int offset)
{
    uint8_t* p = u8_Tx_Data;
		p = p + offset;
    int l = -1;
    int i = 1;
		int neg =0;
    while ( i!=0)
    {
				i = a/muoi_mu(++l);
    }
    if (a == 0) l =1;
    
    if (a < 0)
    {
       *p = '-';
       p++;
       a = -a;
			 neg =1;
    }
    int temp;
    for (int j =0 ; j < l ; j++)
      {
          temp =a;
					temp = temp%muoi_mu(j+1);
          *(p+l-1-j) =temp/muoi_mu(j) + 48;
          a = a - temp;
      }
		
    return l+neg;
}
//

void tran_mess( uint8_t mode)
{
	/* Data Process*/
	u8_Tx_Data[0] = '!';
		if (mode == Tran_V)
	{
		/* Send velocity*/
		u8_Tx_Data[1] = 'V';
		u8_Tx_Data[2] = ';';
		int l  =  Int2Str(rate1,3) +3;
		u8_Tx_Data[l++] = ';';
		l = l + Int2Str(rate2,l);
		u8_Tx_Data[l++] = '#' ; 
		HAL_UART_Transmit_DMA(&huart4,u8_Tx_Data,l);
	}
	else if (mode == Tran_P)
	{
		/* Send Counter*/
		u8_Tx_Data[1] = 'P';
		u8_Tx_Data[2] = ';';
		int temp = encoder_cnt1,temp1, t =3;
		int l = Int2Str(encoder_cnt1,3) +3 ;
		
		u8_Tx_Data[l++] = ';';
		l = l + Int2Str(encoder_cnt2,l);
		u8_Tx_Data[l++] = '#' ; 
		HAL_UART_Transmit_DMA(&huart4,u8_Tx_Data,l);
	}

}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
		
	if (htim -> Instance == TIM1)
	{
			if (control_mode == Control_V)
			{		
			/* Mortor 1*/
		
			encoder_cnt_pre1 = encoder_cnt1;
			encoder_cnt1 = __HAL_TIM_GetCounter(&htim2);
			pulse1 = encoder_cnt1 - encoder_cnt_pre1;
			rate1 = pulse1*3000/374;
			if (s_rate1 != 0)
		{
			/* Compute PID Output for mortor1*/
			e1 = s_rate1 -rate1;

			static double u1_1 = 0 , u1 =0,e1_1 = 0,e2_1 =0;
			u1= u1_1 + Kp1*(e1-e1_1) + Ki1*Ts*(e1_1+e1)/2 + Kd1*(e1-2*e1_1+e2_1)/Ts;
			pid1 = u1;
			if (pid1 -u1 >-0.5) pid1++;
			e2_1 = e1_1;
			e1_1 = e1;
			u1_1=pid1;
			
			
			if (pid1 >= 0)
			{
			if (pid1 > x) pid1 =x;
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pid1);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,0);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,1);	
			}
			else
			{
			if (pid1 < - x) pid1 = - x;
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2, - pid1);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,1);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,0);
			}
		}
		else  /* Stop motor 1*/
		{
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,0);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,0);
		}
		/* Mortor 2*/

			encoder_cnt_pre2 = encoder_cnt2;
			encoder_cnt2 = __HAL_TIM_GetCounter(&htim5);
			pulse2 = encoder_cnt2 - encoder_cnt_pre2;
			rate2 = pulse2*3000/374;
			e2 = s_rate2 -rate2;
					
		if (s_rate2 !=0)
		{
			/* Compute PID Output for mortor 2*/
			static double u1_2 = 0 , u2 =0,e1_2 = 0,e2_2 =0;
			u2= u1_2 + Kp2*(e2-e1_2) + Ki2*Ts*(e1_2+e2)/2 + Kd2*(e2-2*e1_2+e2_2)/Ts;
			pid2 = u2;
			if (pid2 -u2>0.5) pid2++;
			e2_2 = e1_2;
			e1_2 = e2;
			u1_2=pid2;
			

			if (pid2 >= 0)
			{
			if (pid2 > x) pid2 =x;
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pid2);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,0);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,1);	
			}
			else
			{
			if (pid2 < - x) pid2 = - x;
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, - pid2);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,1);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,0);
			}
		}
		else
		{
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,0);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,0);

		}
		}
		else if (control_mode == Control_P)
		{
			/*Position control*/
			/*Motor 1*/
			encoder_cnt1 = __HAL_TIM_GetCounter(&htim2);
			static int e_1_1 =0,e_1_2= 0,e_1=0,pid_1;
			static double u_1_1=0 ,u_1 = 0;
			e_1 = s_pos1 - encoder_cnt1 ;
			u_1= u_1_1 + Kp_1*(e_1-e_1_1) + Ki_1*Ts*(e_1_1+e_1)/2 + Kd_1*(e_1-2*e_1_1+e_1_2)/Ts;
			e_1_2 = e_1_1;
			e_1_1 = e_1;
			pid_1 =u_1;
			u_1_1 = pid1;
			
			if (pid_1 -u_1 >-0.5) pid_1++;
			pid1 = pid_1 ;
			
			if (pid1 >= 0)
			{
			if (pid1 > x) pid1 =x;
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pid1);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,0);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,1);	
			}
			else
			{
			if (pid1 < - x) pid1 = - x;
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2, - pid1);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,1);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,0);
			}
			/* Motor 2*/
			encoder_cnt2 = __HAL_TIM_GetCounter(&htim5);
			static int e_2_1 =0,e_2_2= 0,e_2=0,pid_2;
			static double u_2_1=0 ,u_2 = 0;
			e_2 = s_pos2 - encoder_cnt2 ;
			u_2= u_2_1 + Kp_2*(e_2-e_2_1) + Ki_2*Ts*(e_2_1+e_2)/2 + Kd_2*(e_2-2*e_2_1+e_2_2)/Ts;
			e_2_2 = e_2_1;
			e_2_1 = e_2;
			pid_2 =u_2;
			u_2_1 = pid_2;
			if (pid_2 -u_2 >-0.5) pid_2++;
			//
			pid2 = pid_2;
			if (pid2 >= 0)
			{
			if (pid2 > x) pid2 =x;
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pid2);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,0);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,1);	
			}
			else
			{
			if (pid2 < - x) pid2 = - x;
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, - pid2);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,1);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,0);
			}
		} 
		/* Send Data every two circle Timer1 (10ms)*/
		static uint8_t t =0;
		t ++;
		if (t ==1){
		tran_mess(send_mode);
		t=0;
		}
	}
		
	}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	u8_Rx_Data[start_buff] = u8_Rx_c ;
	start_buff++ ;
	if (start_buff == end_buff)
	{
		/* full buffer*/
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_SET);
		
	}
}

uint8_t Pop_Rx_Data()
{
	end_buff++;
	while (end_buff == start_buff)
	{
		/* empty Buffer*/
		/* Wait for new data*/
		HAL_Delay(1);
	}
	return u8_Rx_Data[end_buff];
}
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
  /* Prevent unused argument(s) compilation warning */
	if (u8_Rx_SPI[0] == '0')
	{
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,1);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,0);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,0);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,0);
		s_rate1 = s_rate2 = 150;
	}
		if (u8_Rx_SPI[0] == '1')
	{
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,1);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,0);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,0);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,0);
		s_rate1 = s_rate2 = -150;
	}
		if (u8_Rx_SPI[0] == '2')
	{
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,1);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,0);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,0);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,0);
		s_rate1 = 50;
		s_rate2 = 100;
	}
		if (u8_Rx_SPI[0] == '3')
	{
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,1);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,0);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,0);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,0);
		s_rate1 = 100;
		s_rate2 = 50;
	}
		if (u8_Rx_SPI[0] == '4')
	{
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,0);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,0);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,0);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,0);
		s_rate2 = 0;
		s_rate1 =0;
		
	}
	//HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_13);
	//HAL_UART_Transmit_IT(&huart1,buff,12);
	HAL_SPI_Receive_IT(&hspi1,u8_Rx_Data,1);
	
  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_SPI_RxCpltCallback should be implemented in the user file
   */
}

int Str2Int(uint8_t* p , uint8_t i)
{   
    int a = 0;
    if (*p != '-')
    {
        for (int j =0; j < i ; j++)
        {
						a += (*(p+j) -48 ) * muoi_mu(i-j-1) ;
        }
    
    }
    else
    {
       for (int j =1; j < i ; j++)
        {
            a += (*(p+j) -48 ) * muoi_mu(i-j-1) ;
        } 
        a = -a ;
    }
    return a;
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
