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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CAN_receive.h"
#include "bsp_can.h"
#include "remote_control.h"
#include "bsp_rc.h"
#include "UART.h"
#include "stdio.h"
#include "stdarg.h"
#include "string.h"
#include "math.h"

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
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart6_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for chassisTask */
osThreadId_t chassisTaskHandle;
const osThreadAttr_t chassisTask_attributes = {
  .name = "chassisTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* USER CODE BEGIN PV */
const RC_ctrl_t *local_rc_ctrl;

PID_preset_t chassisPreset = {3.6, 0.015, 4.2};  //0.8f, 0.025f, 0.95f   {1.6, 0.045, 0.10}; 32 0.42
PID_preset_t yawPresetCurrentRPM = {46, 10.0, 16.4};
PID_preset_t yawPresetVoltageRPM = {50, 8.0, 2.0};
PID_preset_t yawPresetCurrentPosition = {12, 1, 0.0};
PID_preset_t yawPresetVoltagePosition = {84, 0.12, 128.0};
PID_preset_t shooterPreset = {1.20f, 0.12f, 1.60f};
PID_preset_t powerPreset = {0, 0.000000000001, 0};
PID_preset_t customPreset = {0, 0, 0};
PID_preset_t DONUTMOTOR = {50, 8.0, 2.0}; // 46, 10, 16.4

int target = 2000;

can_msg_id_e boardID = CAN_b2b_A_ID;               // remember to change the board ID, board A - chassis, board B - Pitch axis

chassis_motor_config chassis = {{1,2,3,4,0,0,0,0}};
chassis_motor_RPM chassisTargetRPM = {{0,0,0,0,0,0,0,0}};
chassis_motor_current chassisTargetCurrent = {{0,0,0,0,0,0,0,0}};

extern game_status_t game_status;
extern power_heat_data_t power_heat_data;
extern robot_status_t robot_status;
extern b2b_motorCtrl_t b2bMotorCtrl;
extern b2b_gyro_t b2bGyro;


float calcChassisPower = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART3_UART_Init(void);
void TaskMain(void *argument);
void TaskChassis(void *argument);

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
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_SPI1_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  can_filter_init();
  remote_control_init();
  usart_Init();
  local_rc_ctrl = get_remote_control_point();

  //__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(TaskMain, NULL, &defaultTask_attributes);

  /* creation of chassisTask */
  chassisTaskHandle = osThreadNew(TaskChassis, NULL, &chassisTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
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

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_10TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 3;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_10TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 335;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 83;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 249;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  htim8.Init.Period = 19999;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 100000;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_EVEN;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12|GPIO_PIN_11|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pins : PH12 PH11 PH10 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_11|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF15_EVENTOUT;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int16_t positionPIDByMe(int8_t *isNegativeRegion1, int8_t *previousRegion1, int16_t DifferenceBetweenCurrentAndWannabePosition, int16_t *sumI1, float kPu, float kIu, float kDu) {
	if (DifferenceBetweenCurrentAndWannabePosition >= 0) {
		*isNegativeRegion1 = -1;
	} else {
		*isNegativeRegion1 = 1;
	}
	if (*isNegativeRegion1 != *previousRegion1) {
		*sumI1 = 0;
	}
	*previousRegion1 = *isNegativeRegion1;
	*sumI1 += (int)((float)(DifferenceBetweenCurrentAndWannabePosition)*0.005f);
	int16_t PositionToGo = (int)(kPu*(float)(DifferenceBetweenCurrentAndWannabePosition));
	int16_t IntegralToGo = (int)(kIu*((float)(*sumI1)));
	int16_t DerivativeToGo = (int)((kDu)*((float)(DifferenceBetweenCurrentAndWannabePosition))*(float)200);
	return PositionToGo+IntegralToGo+DerivativeToGo;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_TaskMain */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_TaskMain */
void TaskMain(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, 1);
	// __HAL_TIM_PRESCALER(&htim4, 2);
	// HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	// HAL_TIM_Base_Start(&htim4);
	osDelay(150);
	// __HAL_TIM_PRESCALER(&htim4, 0);
	osDelay(150);
	// HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);

	for(;;) {
		HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, 0);
		//set_motor_voltage(5, 4000);
		osDelay(500);
		//sendB2bData(CAN_b2b_A_ID, 1, 1, 1, 1);
		HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, 1);
		//set_motor_voltage(5, -4000);
		osDelay(500);
		//sendB2bData(CAN_b2b_A_ID, 0, 0, 0, 0);
		//usart_printf("ACTIVE = %d \r\n", power_heat_data.chassis_power);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_TaskChassis */
/**
* @brief Function implementing the chassisTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TaskChassis */
void TaskChassis(void *argument)
{
  /* USER CODE BEGIN TaskChassis */
    int16_t rcRPM[4] = {0,0,0,0};                              // maps rc percentage reading to motors, assuming we're running M3508s at max 469RPM
    int16_t rcPitch = 0;                                   // range: 3376 ~ 2132
    //int16_t targetRPM[4] = {0,0,0,0};
  /* Infinite loop */

    // Test Code
    HAL_TIM_Base_Start(&htim1);
    HAL_TIM_Base_Start(&htim4);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    int8_t motorOn = 0;
    int8_t switched = 0;
    int8_t shot1Round = 0;
    //int8_t RNC = 0;
    int16_t sumI1 =0;
    int16_t sumI2 =0;
    int16_t sumI3 =0;
    int16_t sumI4 =0;
    int8_t isNegativeRegion1 = 0;
    int8_t isNegativeRegion2 = 0;
    int8_t isNegativeRegion3 = 0;
    int8_t isNegativeRegion4 = 0;
    int8_t previousRegion1 = 0;
    int8_t previousRegion2 = 0;
    int8_t previousRegion3 = 0;
    int8_t previousRegion4 = 0;
    int16_t shooterMotor = 0;
    // int16_t pR = 0;
    //int8_t counter = 0;
    uint16_t testmotor = 6161;
    uint16_t pivoter = 0;
    double angle = 0;
    int16_t xJoystickDirection = 0;
    int16_t yJoystickDirection = 0;
    int16_t rotationOfChassis = 0;
    // Total Rotation is 1.25 times for 90degrees therefore motor has to rotate
    // PID onto this (This is the hypothetical orientation)
    int16_t chassisOrientation = 0;
    int16_t chassisPID = 0;
    int16_t rcVal2 = 0;

    int16_t randomOrientations[24] = {-380, -202, 462, -114, 240, -210, 150, 170, 248, 106, 118, 538, -260, -288, -120, 86, -264, 452, -592, 390, -410, 414, 54, -542};
    int16_t startingVal = 0;
    int8_t started = 0;
    uint8_t increment = 0;
    int16_t instancesCounter = 0;

    int16_t previousVal = 0;
    int32_t rotationalVal = 0;
    int16_t revolutions = 0;
    int8_t resetPerStart = 0;
    int32_t rotationTarget = 0;
    int32_t posForGunMotor = 0;
    int8_t burst = 3;

    int8_t customFiringModeSwitcher = 0;
    int8_t startedChecking = 0;
    int8_t switchedDown = 0;
    int8_t finalTHing = 0;
    int16_t counterForSwitching = 0;

    int16_t buzzLengthCounter = 0;
    int8_t beepingInProgress = 0;
    int8_t beeped = 1;

    for(;;) {
	    for (int i = 0; i < 4; i++) {
	        rcRPM[i] = getRCchannel(i) * 13.645f;              // 13.645 = 469 / 187 / 660 * 3591, 660 = max reading in one direction
	    }
	    rcPitch = getRCchannel(1) * 0.94f + 2754;
	    int16_t leftDial = getRCchannel(4);

		int8_t chassisTurning = getRCswitch(1);

		float funnyKP = 0.022;
		float funnyKI = -0.02;
		float funnyKD = 0.00005;
		float rotationScalar = -540; //-540

		if (counterForSwitching > 200) {
			startedChecking = 0;
			switchedDown = 0;
			finalTHing = 0;
			counterForSwitching = 0;
		}

		if (startedChecking == 1) {
			counterForSwitching++;
		}

		int8_t movementUpOrDown = 5;
		if (increment == 18) {
			increment = 0;
		}

		if (instancesCounter > 100) {
			increment++;
			instancesCounter = 0;
		}

		if (chassisTurning == 1 && startedChecking == 0) {
			counterForSwitching = 0;
			startedChecking = 1;

		}
		if (chassisTurning == 3 && startedChecking == 1) {
			switchedDown = 1;
		}
		if (chassisTurning == 1 && startedChecking == 1 && switchedDown == 1) {
			finalTHing = 1;
		}

		if (chassisTurning == 3 && startedChecking == 1 && switchedDown == 1 && finalTHing == 1 && counterForSwitching < 200) {
			switchedDown = 0;
			startedChecking = 0;
			counterForSwitching = 0;
			finalTHing = 0;
			customFiringModeSwitcher++;
			beeped = 0;
			buzzLengthCounter = 0;

		}
		if (customFiringModeSwitcher > 2) {
			customFiringModeSwitcher = 0;
		}


		// usart_printf("$%d %d %d %d\r\n;",customFiringModeSwitcher, startedChecking, switchedDown, counterForSwitching);

		if (beeped == 0) {
			switch (customFiringModeSwitcher) {
			case 0:
				if ((buzzLengthCounter == 0)) {
					htim4.Instance->CCR3=150;
					HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, 1);
				}
				else if (buzzLengthCounter >=60) {
					beeped = 1;
					htim4.Instance->CCR3=0;
					HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, 0);
				}
				buzzLengthCounter++;
				break;
			case 1:
				if (buzzLengthCounter == 0) {
					htim4.Instance->CCR3=150;
					HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, 1);
				} else if (buzzLengthCounter >=12) {
					beeped = 1;
					htim4.Instance->CCR3=0;
					HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, 0);
				}
				buzzLengthCounter++;
				break;
			case 2:
				if ((buzzLengthCounter == 0) || (buzzLengthCounter == 30) || (buzzLengthCounter == 60)) {
					htim4.Instance->CCR3=150;
					HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, 1);
				} else if ((buzzLengthCounter == 15) || (buzzLengthCounter == 45)) {
					htim4.Instance->CCR3=0;
					HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, 0);
				} else if (buzzLengthCounter >=75) {
					beeped = 1;
					htim4.Instance->CCR3=0;
					HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, 0);
				}
				buzzLengthCounter++;
				break;
			}
		}



		switch (chassisTurning) {
		case 1:
			xJoystickDirection = rcRPM[2]*cos(angle) - rcRPM[3]*sin(angle);
			yJoystickDirection = rcRPM[2]*sin(angle) + rcRPM[3]*cos(angle);
			rotationOfChassis = rcRPM[0];


			started = 0;



			break;
		case 2:
			int8_t delta = 5;
			if (started == 0) {
				startingVal = 0;
				started = 1;
			}
			if (startingVal >= randomOrientations[increment]-delta && startingVal <= randomOrientations[increment]+delta) {
				instancesCounter++;
			} else if (startingVal > randomOrientations[increment]-delta) {
				startingVal -= movementUpOrDown;
			} else {
				startingVal += movementUpOrDown;
			}


			xJoystickDirection = rcRPM[2]*cos(angle) - rcRPM[3]*sin(angle);
			yJoystickDirection = rcRPM[2]*sin(angle) + rcRPM[3]*cos(angle);
			int16_t hypotheticalP = funnyKP*(startingVal - chassisOrientation);
			if (hypotheticalP >= 0) {
				isNegativeRegion3 = -1;
			} else {
				isNegativeRegion3 = 1;
			}
			if (hypotheticalP != previousRegion3) {
				sumI3 = 0;
			}
			previousRegion3 = hypotheticalP;
			sumI3 += (startingVal - chassisOrientation)*0.005;
			int16_t hypotheticalI = funnyKI*(sumI3);
			int16_t hypotheticalD = funnyKD*(startingVal - chassisOrientation)*200;
			chassisPID = hypotheticalP + hypotheticalI + hypotheticalD;
			chassisOrientation += chassisPID;
			rotationOfChassis = rcRPM[0]+rotationScalar*chassisPID;

			/*
			xJoystickDirection = rcRPM[2]*cos(angle) - rcRPM[3]*sin(angle);
			yJoystickDirection = rcRPM[2]*sin(angle) + rcRPM[3]*cos(angle);
			int16_t hypotheticalP = funnyKP*(leftDial - chassisOrientation);
    		if (hypotheticalP >= 0) {
				isNegativeRegion3 = -1;
			} else {
				isNegativeRegion3 = 1;
			}
			if (hypotheticalP != previousRegion3) {
				sumI3 = 0;
			}
			previousRegion3 = hypotheticalP;
			sumI3 += (leftDial - chassisOrientation)*0.005;
			int16_t hypotheticalI = funnyKI*(sumI3);
			int16_t hypotheticalD = funnyKD*(leftDial - chassisOrientation)*200;
			chassisPID = hypotheticalP + hypotheticalI + hypotheticalD;
			chassisOrientation += chassisPID;
			rotationOfChassis = rcRPM[0]+rotationScalar*chassisPID;
			*/

			break;
		default:
			started = 0;
			xJoystickDirection = rcRPM[2];
			yJoystickDirection = rcRPM[3];
			rotationOfChassis = rcRPM[0];
		}
		// int16_t chassisConvert = ((-1*(chassisOrientation))*3.32f)+4755;


		chassisTargetRPM.motorRPM[0] = yJoystickDirection + rotationOfChassis + xJoystickDirection;
		chassisTargetRPM.motorRPM[1] = yJoystickDirection + rotationOfChassis - xJoystickDirection;
		chassisTargetRPM.motorRPM[2] = -yJoystickDirection + rotationOfChassis - xJoystickDirection;
		chassisTargetRPM.motorRPM[3] = -yJoystickDirection + rotationOfChassis + xJoystickDirection;

	    //usart_printf("%d\r\n", targetMotorRPM.motorRPM[0]);
	   // calcChassisPower = (float)power_heat_data.chassis_voltage * (float)power_heat_data.chassis_current / (float)1000000;

	    //if (calcChassisPower >= 30) {
	    //chassisTargetCurrent = applyPowerlimit(chassis, chassisTargetRPM, calcChassisPower);

	    //CAN1_cmd_b2b(CAN_b2b_A_ID, 1, 1, 1, 1);

		float kPg = 0.1;
		float kIg = 0;
		float kDg = 0;
		if (customFiringModeSwitcher == 1) {
			burst = 1;
		} else if (customFiringModeSwitcher == 2) {
			burst = 3;
		}





		setM3508RPM(1, chassisTargetRPM.motorRPM[0], chassisPreset);
		setM3508RPM(2, chassisTargetRPM.motorRPM[1], chassisPreset);
		setM3508RPM(3, chassisTargetRPM.motorRPM[2], chassisPreset);
		setM3508RPM(4, chassisTargetRPM.motorRPM[3], chassisPreset);

		int16_t roundsPerSecond = 20;
	    // Constant SHOULD BE 1.3636, 0.08
		int8_t rcSwitchToShoot = getRCswitch(0);
	    if (rcSwitchToShoot == 1) {
	    	if (customFiringModeSwitcher == 0) {
	    		setM3508RPM(5, roundsPerSecond * 270, chassisPreset);
	    	} else {
				shooterMotor = getMotorPosition(5);
				// Resets the total rotationValue to avoid going too high
				if (resetPerStart == 0) {
					int32_t rotationTarget1 = (36860 * burst);// + shooterMotor
					rotationTarget = rotationTarget1 + shooterMotor;

					revolutions = 0;
					rotationalVal = shooterMotor;
					previousVal = shooterMotor;
				}
				resetPerStart = 1;
				posForGunMotor = kPg*(rotationTarget - rotationalVal);

				// counts the amount of rotations
				if ((shooterMotor - previousVal) < -1000) {
					 revolutions++;
				} /* else if ((shooterMotor - previousVal) > 4500) {
				revolutions --
				}
				*/
				previousVal = shooterMotor;
				rotationalVal = shooterMotor + (revolutions*8191);




				// int16_t PositionToGo = kPu*(rcVal-testmotor);
				if (posForGunMotor >= 0) {
					isNegativeRegion4 = -1;
				} else {
					isNegativeRegion4 = 1;
				}
				if (posForGunMotor != previousRegion4) {
					sumI4 = 0;
				}
				previousRegion4 = posForGunMotor;
				sumI4 += (rotationTarget-testmotor)*0.005;
				int32_t IntegralToGo4 = kIg*(sumI4);
				int32_t DerivativeToGo4 = kDg*(rotationTarget-testmotor)*200;

				int32_t finalRPM = 0;
				if ((posForGunMotor+IntegralToGo4+DerivativeToGo4) > (roundsPerSecond * 270)) {
					finalRPM =roundsPerSecond * 270;
				} else {
					finalRPM = posForGunMotor+IntegralToGo4+DerivativeToGo4;
				}
				setM3508RPM(5, finalRPM, chassisPreset);
	    	}
	    	switched = 0;
	    	/*
	    	switched = 0;
	    	setM3508RPM(5, roundsPerSecond * 270, chassisPreset);
	    	*/
	    	// __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, rcVal + 252);
	    	// __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, rcVal + 252);
	    } else {
	    	resetPerStart = 0;
	    	setM3508RPM(5, 0, chassisPreset);
	    }
	    if (rcSwitchToShoot == 2 && switched == 0 && motorOn == 0) {
	    	motorOn = 1;
	    	switched = 1;
	    } else if (rcSwitchToShoot == 2 && switched == 0 && motorOn == 1) {
	    	motorOn = 0;
	    	switched = 1;
	    }
	    if (rcSwitchToShoot == 3) {
	    	switched = 0;
	    }

	    // MAX SPEED = 759
    	htim1.Instance->CCR1=200+(500*motorOn);
    	htim1.Instance->CCR2=200+(500*motorOn);

    	// int32_t dividedRotation = rotationalVal * 0.01f;
    	// usart_printf("$%d %d %d\r\n;",rotationalVal, shooterMotor, rotationTarget);
    	testmotor = getMotorPosition(6);

    	// Turret Slope min, centre, max : 5573, 6161, 6751
    	// min = 5600
    	// max = 6700
    	//(These are ABSOLUTE MAXES)
    	// Difference = 0, 589, 1178

    	float kPu =0.005; // 0.005
    	float kIu =0.0001; // -0.0001
    	float kDu =0.0005; // 0.0005



    	int16_t rcVal = (getRCchannel(1)*0.88f)+6161;

    	int16_t DifferenceBetweenCurrentAndWannabePosition = rcVal-testmotor;

    	if (testmotor < 5500) {
    		setGM6020voltageRPM(6, 5, DONUTMOTOR);
    	} else if (testmotor > 6800) {
    		setGM6020voltageRPM(6, -5, DONUTMOTOR);
    	} else {

    		//positionPIDByMe(int8_t &negativeRegion, int8_t &previousRegion, int16_t &differenceBetween, int16_t &integralVal float &kPVal, float &kIVal, float &kDVal);
    		// positionPIDByMe(isNegativeRegion1, previousRegion1, DifferenceBetweenCurrentAndWannabePosition, sumI1, kPu, kIu, kDu);
    		/*
    		if (PositionToGo >= 0) {
				isNegativeRegion1 = -1;
			} else {
				isNegativeRegion1 = 1;
			}
			if (PositionToGo != previousRegion1) {
				sumI1 = 0;
			}
			previousRegion1 = isNegativeRegion1;
			sumI1 += (DifferenceBetweenCurrentAndWannabePosition)*0.005;
			int16_t PositionToGo = kPu*(DifferenceBetweenCurrentAndWannabePosition);
			int16_t IntegralToGo = kIu*(sumI1);
			int16_t DerivativeToGo = kDu*(DifferenceBetweenCurrentAndWannabePosition)*200;
			*/

			setGM6020voltageRPM(6, positionPIDByMe(&isNegativeRegion1, &previousRegion1, DifferenceBetweenCurrentAndWannabePosition, &sumI1, kPu, kIu, kDu), DONUTMOTOR);
			// usart_printf("$%d %d %d\r\n;", PositionToGo, IntegralToGo, DerivativeToGo);
    	}


    	// 2524-6986
    	// 90 degree = 2691, 6799
    	pivoter = getMotorPosition(7);
    	angle = ((pivoter-4755)*0.00024343f)*3.14159265f;

    	float kPr =0.022; // 0.001
		float kIr =0.02; // -0.02
		float kDr =0.00005; // 0.00015

		if (chassisTurning == 2) {
			// rcVal2 = ((-1*(leftDial))*3.32f)+4755;

			rcVal2 = ((-1*(startingVal))*3.32f)+4755;
		} else {
			rcVal2 = (leftDial*3.32f)+4755;
		}


		int16_t DiffOfTurret = rcVal2-pivoter;

		if (pivoter < 2400) {
			setGM6020voltageRPM(7, 5, DONUTMOTOR);
		} else if (pivoter > 7100) {
			setGM6020voltageRPM(7, -5, DONUTMOTOR);
		} else {
			/*
			if (DiffOfTurret >= 0) {
				isNegativeRegion2 = -1;
			} else {
				isNegativeRegion2 = 1;
			}
			if (isNegativeRegion2 != previousRegion2) {
				sumI2 = 0;
			}
			previousRegion2 = PositionToGo2;
			sumI2 += (DiffOfTurret)*0.005;
			int16_t PositionToGo2 = kPr*(DiffOfTurret);
			int16_t IntegralToGo2 = kIr*(sumI2);
			int16_t DerivativeToGo2 = kDr*(DiffOfTurret)*200;
			*/
			setGM6020voltageRPM(7, positionPIDByMe(&isNegativeRegion2, &previousRegion2, DiffOfTurret, &sumI2, kPr, kIr, kDr), DONUTMOTOR);
			usart_printf("$%d %d\r\n;", DiffOfTurret, sumI2);
		}

		// usart_printf("$%d %d %d\r\n;", rcVal2, pivoter, chassisConvert);
		//usart_printf("$%d\r\d;", rotationOfChassis);
    	// usart_printf("$%d %d\r\n;",rcVal2 ,pivoter);
    	// usart_printf("$%d\r\n;",pivoter);
    	// usart_printRC();

		// sendB2bData(CAN_b2b_A_motorCtrl_ID, rcPitch, 0, 0, 0);

      /*
	    if (boardID == CAN_b2b_B_ID) {
	    	setGM6020voltagePosition(9, b2bMotorCtrl.motor1_Ctrl, yawPresetVoltagePosition);
	    	//setGM6020voltageRPM(9, 100, yawPresetVoltageRPM);
	    	//CAN2_cmd_motors(CAN_GROUP3C_ID, 5000, 0, 0, 0);
	    	sendB2bData(CAN_b2b_B_gyro_ID, b2bMotorCtrl.motor1_Ctrl, getMotorPosition(9), 0, 0);
	    }
      */
	    //set_GM6020_current(5, 8000);               getMotorPosition(9) getMotorCurrent(9)
	    //set_GM6020_voltage(5, 2000);
	    //setGM6020voltageRPM(5, 100, yawPresetVoltageRPM);
	    //setGM6020currentPosition(5, target, yawPresetPosition);
	    //setGM6020voltagePosition(5, target, yawPresetVoltagePosition);
	    //usart_printf("%d\r\n", getMotorRPM(5));
	    //usart_printRC();
	    //sprintf((char*)txbuf, "%f \r\n", power_heat_data.chassis_power);
	    //HAL_UART_Transmit(&huart1, txbuf, strlen((char*)txbuf), HAL_MAX_DELAY);
	    //txbuf = *((float*)&power_heat_data.chassis_power);
	    //usart_printf("%f %d\r\n", calcChassisPower, 30);

        osDelay(5);
    }
  /* USER CODE END TaskChassis */
}

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
