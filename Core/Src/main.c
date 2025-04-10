/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stdio.h"
#include "string.h"
#include "../UserCode/Motor/motor.h"
#include "../UserCode/PID/pid.h"
#include "../UserCode/Serial/serial.h"
//#include "../UserCode/Motion/motion.h"
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
//TIM_HandleTypeDef htim1;
//TIM_HandleTypeDef htim2;
//TIM_HandleTypeDef htim3;
//TIM_HandleTypeDef htim4;

//UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */
Motor_t tmotor1;
Motor_t tmotor2;

extern PROCESS_t tprocess;
PID_CONTROL_t tpid_ctrl1;
PID_CONTROL_t tpid_ctrl2;
extern char scmd[4];
extern float fset_point;
extern float fset_point1;
extern float fset_point2;
extern float fkp;
extern float fki;
extern float fkd;

extern float fkp1;
extern float fki1;
extern float fkd1;
extern float fkp2;
extern float fki2;
extern float fkd2;
uint8_t pid_init_flag = 0;

uint8_t rxB2B;
//uint8_t rxRawBuf[30] = { NULL };
uint8_t rxRawBuf[60] = { NULL };

volatile int8_t ptr;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void MX_TIM1_Init(void);
void MX_TIM4_Init(void);
void MX_USART3_UART_Init(void);
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  motor_init(&tmotor1, 1);
  motor_init(&tmotor2, 2);
  pid_init(&tpid_ctrl1, 1, 0.5, 0, PID_CONTROLLER_LIMIT_MAX, PID_CONTROLLER_LIMIT_MIN, SAMPLING_TIME);
  pid_init(&tpid_ctrl2, 1, 0.5, 0, PID_CONTROLLER_LIMIT_MAX, PID_CONTROLLER_LIMIT_MIN, SAMPLING_TIME);
  serial_init();
  HAL_UART_Receive_DMA(&huart3, &rxB2B, 1);
//	HAL_UART_Receive_IT(&huart3, &rxB2B, 20);

  //  pid_tunning_set(&tpid_ctrl1, fkp, fki, fkd)
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
   {
     /* USER CODE END WHILE */

     /* USER CODE BEGIN 3 */
 	    switch (tprocess)
 	    {
 	    case NONE:
 	    	motor_deinit(&tmotor1, 1);
 	    	motor_deinit(&tmotor2, 2);

 	    	tmotor1.fposition = 0;
 	    	tmotor2.fposition = 0;
 	    	tmotor1.freference_position = 0;
 	    	tmotor2.freference_position = 0;
 	    	  motor_init(&tmotor1, 1);
 	    	  motor_init(&tmotor2, 2);
//			  motor_reset(&tmotor1); //moi them
//			  motor_reset(&tmotor2); //moi them
 	      break;
 	    case SPID:
          tprocess = NONE;
 	      break;
 	    case VTUN:
 	      break;
 	    case PTUN:
 	      break;
 	    case STOP:
 	      tprocess = NONE;
 	      break;
 	    }
   }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)	// 10ms = 0.01s
{
	if (htim->Instance == htim3.Instance)
	{
			motor_read_encoder(&tmotor1, &ENCODER_TIMER1);
			motor_read_encoder(&tmotor2, &ENCODER_TIMER2);
		  switch (tprocess) {
			case NONE:
				break;
			case SPID:
				break;
			case VTUN:
				motor_set_velocity(&tmotor1, &tpid_ctrl1, fset_point1);
				motor_set_velocity(&tmotor2, &tpid_ctrl2, fset_point2);
//				serial_write_com(scmd, tmotor1.fvelocity, tpid_ctrl1.fkp, tpid_ctrl1.fki, tpid_ctrl1.fkd); //Transmit o day thi ok vi khong bị interrupt làm gián đoạn
				break;
			case PTUN:
				motor_set_position(&tmotor1, &tpid_ctrl1, fset_point1);
				motor_set_position(&tmotor2, &tpid_ctrl2, fset_point2);

//				serial_write_com(scmd, tmotor1.fposition, tpid_ctrl1.fkp, tpid_ctrl1.fki, tpid_ctrl1.fkd); //Transmit o day thi ok vi khong bị interrupt làm gián đoạn
				break;
			case STOP:
				break;
//			case RSET:
//				break;
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == huart3.Instance)
	{
		rxRawBuf[ptr] = rxB2B;
		ptr++;
		if (rxRawBuf[ptr - 1] == '\n')
		{
//			sscanf(rxRawBuf, "%s %f %f %f %f %f", scmd, &fset_point1, &fset_point2, &fkp, &fki, &fkd);
			sscanf(rxRawBuf, "%s %f %f %f %f %f %f %f %f", scmd, &fset_point1, &fset_point2, &fkp1, &fki1, &fkd1, &fkp2, &fki2, &fkd2);


			if (StrCompare(scmd, (uint8_t*)"SPID", 4))
			{
			  pid_tunning_set(&tpid_ctrl1, fkp1, fki1, fkd1);
			  pid_tunning_set(&tpid_ctrl2, fkp2, fki2, fkd2);

			  motor_deinit(&tmotor1, 1);
			  motor_deinit(&tmotor2, 2);
			  motor_init(&tmotor1, 1);
			  motor_init(&tmotor2, 2);

			  /* To ensure stop moving */
			  htim2.Instance->CCR1 = 0;
			  htim2.Instance->CCR2 = 0;
			  htim2.Instance->CCR3 = 0;
			  htim2.Instance->CCR4 = 0;

			  tprocess = SPID;
			}
			else if (StrCompare(scmd, (uint8_t*)"VTUN", 4))
			{
			  tmotor1.freference_velocity = fset_point1;
			  tmotor2.freference_velocity = fset_point2;

			  tprocess = VTUN;
			}
			else if (StrCompare(scmd, (uint8_t*)"PTUN", 4))
			{
			  tmotor1.freference_position = fset_point1;
			  tmotor2.freference_position = fset_point2;

			  tprocess = PTUN;
			}
			else if (StrCompare(scmd, (uint8_t*)"STOP", 4))
			{
			  motor_deinit(&tmotor1, 1);
			  motor_deinit(&tmotor2, 2);
			  pid_reset(&tpid_ctrl1);

			  pid_reset(&tpid_ctrl2);
			  motor_reset(&tmotor1); //moi them
			  motor_reset(&tmotor2); //moi them

			  tprocess = STOP;
			}
			else if (StrCompare(scmd, (uint8_t*)"NONE", 4))
			{
			  tprocess = NONE;
			}
		    memset(rxRawBuf, '\0', sizeof(rxRawBuf));
		    ptr = 0;
		}
		if (strlen(rxRawBuf) > 50)		    memset(rxRawBuf, '\0', sizeof(rxRawBuf));
	}
//	HAL_UART_Receive_IT(&huart3, &rxB2B, 20);
	HAL_UART_Receive_DMA(&huart3, &rxB2B, 1);
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
//static void MX_TIM1_Init(void)
//{
//
//  /* USER CODE BEGIN TIM1_Init 0 */
//
//  /* USER CODE END TIM1_Init 0 */
//
//  TIM_Encoder_InitTypeDef sConfig = {0};
//  TIM_MasterConfigTypeDef sMasterConfig = {0};
//
//  /* USER CODE BEGIN TIM1_Init 1 */
//
//  /* USER CODE END TIM1_Init 1 */
//  htim1.Instance = TIM1;
//  htim1.Init.Prescaler = 0;
//  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim1.Init.Period = 65535;
//  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim1.Init.RepetitionCounter = 0;
//  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
//  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
//  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
//  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
//  sConfig.IC1Filter = 0;
//  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
//  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
//  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
//  sConfig.IC2Filter = 0;
//  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN TIM1_Init 2 */
//
//  /* USER CODE END TIM1_Init 2 */
//
//}
//
///**
//  * @brief TIM2 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_TIM2_Init(void)
//{
//
//  /* USER CODE BEGIN TIM2_Init 0 */
//
//  /* USER CODE END TIM2_Init 0 */
//
//  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
//  TIM_MasterConfigTypeDef sMasterConfig = {0};
//  TIM_OC_InitTypeDef sConfigOC = {0};
//
//  /* USER CODE BEGIN TIM2_Init 1 */
//
//  /* USER CODE END TIM2_Init 1 */
//  htim2.Instance = TIM2;
//  htim2.Init.Prescaler = 719;
//  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim2.Init.Period = 99;
//  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sConfigOC.OCMode = TIM_OCMODE_PWM1;
//  sConfigOC.Pulse = 0;
//  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN TIM2_Init 2 */
//
//  /* USER CODE END TIM2_Init 2 */
//  HAL_TIM_MspPostInit(&htim2);
//
//}
//
///**
//  * @brief TIM3 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_TIM3_Init(void)
//{
//
//  /* USER CODE BEGIN TIM3_Init 0 */
//
//  /* USER CODE END TIM3_Init 0 */
//
//  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
//  TIM_MasterConfigTypeDef sMasterConfig = {0};
//
//  /* USER CODE BEGIN TIM3_Init 1 */
//
//  /* USER CODE END TIM3_Init 1 */
//  htim3.Instance = TIM3;
//  htim3.Init.Prescaler = 7199;
//  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim3.Init.Period = 99;
//  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN TIM3_Init 2 */
//
//  /* USER CODE END TIM3_Init 2 */
//
//}
//
///**
//  * @brief TIM4 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_TIM4_Init(void)
//{
//
//  /* USER CODE BEGIN TIM4_Init 0 */
//
//  /* USER CODE END TIM4_Init 0 */
//
//  TIM_Encoder_InitTypeDef sConfig = {0};
//  TIM_MasterConfigTypeDef sMasterConfig = {0};
//
//  /* USER CODE BEGIN TIM4_Init 1 */
//
//  /* USER CODE END TIM4_Init 1 */
//  htim4.Instance = TIM4;
//  htim4.Init.Prescaler = 0;
//  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim4.Init.Period = 65535;
//  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
//  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
//  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
//  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
//  sConfig.IC1Filter = 0;
//  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
//  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
//  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
//  sConfig.IC2Filter = 0;
//  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN TIM4_Init 2 */
//
//  /* USER CODE END TIM4_Init 2 */
//
//}
//
///**
//  * @brief USART3 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_USART3_UART_Init(void)
//{
//
//  /* USER CODE BEGIN USART3_Init 0 */
//
//  /* USER CODE END USART3_Init 0 */
//
//  /* USER CODE BEGIN USART3_Init 1 */
//
//  /* USER CODE END USART3_Init 1 */
//  huart3.Instance = USART3;
//  huart3.Init.BaudRate = 115200;
//  huart3.Init.WordLength = UART_WORDLENGTH_8B;
//  huart3.Init.StopBits = UART_STOPBITS_1;
//  huart3.Init.Parity = UART_PARITY_NONE;
//  huart3.Init.Mode = UART_MODE_TX_RX;
//  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
//  if (HAL_UART_Init(&huart3) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN USART3_Init 2 */
//
//  /* USER CODE END USART3_Init 2 */
//
//}
//
/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
//static void MX_GPIO_Init(void)
//{
///* USER CODE BEGIN MX_GPIO_Init_1 */
///* USER CODE END MX_GPIO_Init_1 */
//
//  /* GPIO Ports Clock Enable */
//  __HAL_RCC_GPIOD_CLK_ENABLE();
//  __HAL_RCC_GPIOA_CLK_ENABLE();
//  __HAL_RCC_GPIOB_CLK_ENABLE();
//
///* USER CODE BEGIN MX_GPIO_Init_2 */
///* USER CODE END MX_GPIO_Init_2 */
//}
//
///* USER CODE BEGIN 4 */
//
///* USER CODE END 4 */

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
