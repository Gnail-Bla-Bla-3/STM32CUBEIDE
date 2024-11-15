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
//#include "CAN_receive.h"
#include "CAN.h"
#include "bsp_can.h"
#include "remote_control.h"
#include "bsp_rc.h"
#include "UART.h"
#include "stdio.h"
#include "stdarg.h"
#include "string.h"
#include "BMI088driver.h"
#include "pid.h"
#include "bsp_imu_pwm.h"
#include "ist8310driver.h"
#include "pwm.h"
#include "songs.h"
#include "buffer.h"
#include "math.h"
#include "driving.h"
#include "I2C.h"
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

I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim10;
DMA_HandleTypeDef hdma_tim4_ch3;
DMA_HandleTypeDef hdma_tim5_ch1;
DMA_HandleTypeDef hdma_tim5_ch2;

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
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for chassisTask */
osThreadId_t chassisTaskHandle;
const osThreadAttr_t chassisTask_attributes = {
  .name = "chassisTask",
  .stack_size = 1124 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for IMUtempPIDtask */
osThreadId_t IMUtempPIDtaskHandle;
const osThreadAttr_t IMUtempPIDtask_attributes = {
  .name = "IMUtempPIDtask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for turretTask */
osThreadId_t turretTaskHandle;
const osThreadAttr_t turretTask_attributes = {
  .name = "turretTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* USER CODE BEGIN PV */
const RC_ctrl_t *local_rc_ctrl;
//3.6 0.015, 4.2
PID_preset_t chassisPreset = {3.6, 0, 0};  //0.8f, 0.025f, 0.95f   {1.6, 0.045, 0.10}; 32 0.4
PID_preset_t yawPresetCurrentRPM = {46, 10.0, 16.4};


PID_preset_t yawPresetVoltageRPM = {300, 0, 0};


PID_preset_t yawPresetCurrentPosition = {12, 1, 0.0};
PID_preset_t yawPresetVoltagePosition = {36, 0.3, 0.0};

int16_t joystickTunner = 0;

// 50, 0, 110
PID_preset_t pitchPresetVoltagePosition = {70.0, 0.0, 421.0};
PID_preset_t indexerPreset = {1.1f, 0.02f, -2.4f};
PID_preset_t shooterPreset = {1.20f, 0.12f, 1.60f};
PID_preset_t powerPreset = {0, 0.000000000001, 0};
PID_preset_t flywheel = {12, 0, 0};
PID_preset_t customPreset = {0, 0, 0};

int target = 2000;
float JoulesBuffer = 60;
float maxPower = 0;

uint8_t currentGameStatus = 0;
uint8_t calibration = 0;
uint8_t rotationTime = 0;
uint8_t shootingPhase = 0;
uint8_t songPhase = 0;
uint8_t firingStages = 0;
uint8_t SHOOT = 0;

uint8_t setY = 70;
uint8_t setX = 120;



float mouseXScaler = 0.85;
float mouseYScaler = 0.17;

extern game_status_t game_status;
extern power_heat_data_t power_heat_data;
extern robot_status_t robot_status;
extern pc_control_t pc_control;
extern cv_receive_t cv_receive;


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
static void MX_TIM10_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM5_Init(void);
void TaskMain(void *argument);
void TaskChassis(void *argument);
void imu_temp_control_task(void *argument);
void TaskTurret(void *argument);

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
  MX_TIM10_Init();
  MX_I2C3_Init();
  MX_TIM5_Init();
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

  /* creation of IMUtempPIDtask */
  IMUtempPIDtaskHandle = osThreadNew(imu_temp_control_task, NULL, &IMUtempPIDtask_attributes);

  /* creation of turretTask */
  turretTaskHandle = osThreadNew(TaskTurret, NULL, &turretTask_attributes);

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
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 400000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
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
  htim1.Init.Prescaler = 83;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
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
  htim4.Init.Period = 20999;
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
  sConfigOC.Pulse = 10499;
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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 83;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

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
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 0;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 4999;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */
  HAL_TIM_MspPostInit(&htim10);

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
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : PG6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PG3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : INT1_ACCEL_Pin_Pin INT1_GRYO_Pin_Pin */
  GPIO_InitStruct.Pin = INT1_ACCEL_Pin_Pin|INT1_GRYO_Pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
	int16_t aimingCounter = 0;
	int16_t firingCounter = 0;

	// LED_PWM_Start();
	// HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, 1);
	// __HAL_TIM_PRESCALER(&htim4, 2);
	// HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	// HAL_TIM_Base_Start(&htim4);
	if (BMI088_accel_init()) {
		//usart_printf("WARNING - BMI088 accelerometer init failed \r\n");
	}
	if (BMI088_gyro_init()) {
		usart_printf("WARNING - BMI088 gyroscope init failed \r\n");
	}
	if (ist8310_init()) {
		usart_printf("WARNING - IST8310 compass init failed \r\n");
	}
	CAN_defineMotor(Bus2, M3508, 1);
	CAN_defineMotor(Bus2, M2006, 2);
	CAN_defineMotor(Bus2, GM6020, 1);

	osDelay(150);

	// HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
	JoulesBuffer = 60;
	for(;;) {

    	int8_t leftSwitch = getRCswitch(1);
		// currentGameStatus = game_status.game_progress;
		if (game_status.game_progress == 0 && currentGameStatus < 0) {
			currentGameStatus = 0;
		} else if ((game_status.game_progress == 1 && currentGameStatus < 1) || leftSwitch == 2) {
			currentGameStatus = 1;
		} else if (game_status.game_progress == 2 && currentGameStatus < 2) {
			currentGameStatus = 2;
		} else if (game_status.game_progress == 3 && currentGameStatus < 3) {
			currentGameStatus = 3;
		} else if ((game_status.game_progress == 4 && currentGameStatus < 4) || leftSwitch == 3) {
			currentGameStatus = 4;
		} else if ((game_status.game_progress == 5 && currentGameStatus < 5) || leftSwitch == 1) {
			currentGameStatus = 5;
		}
		PWMOutput(LED, 2, 1);
		PWMInitialize(LED, FR, 2, 0.5);
		settingMaxCurrentVal((float)power_heat_data.buffer_energy,100);

		// 0 = red, 1 = blue
		// 0 = SHOOT AT BLUE, 1 = SHOOT AT RED


		switch (currentGameStatus) {
		case 0:
			calibration = 0;
			rotationTime = 0;
			shootingPhase = 0;
			songPhase = 0;
			break;
		case 1:
			calibration = 1;
			rotationTime = 0;
			shootingPhase = 0;
			songPhase = 0;
			break;
		case 2:
			calibration = 1;
			rotationTime = 0;
			shootingPhase = 0;
			songPhase = 1;
			break;
		case 3:
			calibration = 1;
			rotationTime = 0;
			shootingPhase = 0;
			songPhase = 1;
			break;
		case 4:
			calibration = 0;
			rotationTime = 1;
			shootingPhase = 1;
			songPhase = 1;
			break;
		case 5:
			calibration = 0;
			rotationTime = 0;
			shootingPhase = 0;
			songPhase = 0;
			break;
		}

		uint8_t deltaY = 20;
		uint8_t deltaX = 20;

		if (shootingPhase == 1) {
			switch (robot_status.robot_id) {
			case 0:
				if (getBlueX() != 255) {
					aimingCounter = 100;
				} else {
					aimingCounter--;
				}
				if (aimingCounter < 0) {
					aimingCounter = 0;
					firingStages = 0;
				} else if (aimingCounter > 0){
					firingStages = 1;
				}

				if (firingCounter > 0) {
					firingCounter--;
				}
				if (firingCounter == 0) {
					SHOOT = 0;
				} else {
					SHOOT = 1;
				}
				if ((getBlueX() < (setX + deltaX)) && (getBlueX() > (setX - deltaX)) && (getBlueY() < (setY + deltaY)) && (getBlueY() > (setY - deltaY))) {
					firingCounter = 5;
				}

			case 1:
				if (getRedX() != 255) {
					aimingCounter = 100;
				} else {
					aimingCounter--;
				}
				if (aimingCounter < 0) {
					aimingCounter = 0;
					firingStages = 0;
				} else if (aimingCounter > 0){
					firingStages = 1;
				}

				if (firingCounter > 0) {
					firingCounter--;
				}
				if (firingCounter == 0) {
					SHOOT = 0;
				} else {
					SHOOT = 1;
				}
				if ((getRedX() < (setX + deltaX)) && (getRedX() > (setX - deltaX)) && (getRedY() < (setY + deltaY)) && (getRedY() > (setY - deltaY))) {
					firingCounter = 5;
				}
			}
		} else {
			firingStages = 0;
			SHOOT = 0;
		}


		osDelay(5);


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

	PWMInit(&htim1, &htim4, &htim5, &htim8);

    int16_t rcRPM[4] = {0,0,0,0};                              // maps rc percentage reading to motors, assuming we're running M3508s at max 469RPM
    int16_t chassisTargetRPM[4] = {0, 0, 0, 0};


    int16_t rcYaw = 0;
    int16_t chassisrcYaw = 0;



    // int16_t rcPitch = 0;   	                    // range: 2204 ~ 4726


    int8_t jammed = 0;
    int8_t indexerStopped = 1;
    int8_t indexerTargetReached = 0;

    int8_t previousC = 0;

    int8_t bufferDead = 0;
    int8_t M3508Voltage = 24;

    maxPower = 0;
    float scuffedMaxedPower = 0;
    // float percentagePowerLimit = 1;

    float gyroidValue = 0;

    // int8_t switchForShooter = 0;
    int8_t motorOn = 0;

    float MC[4] = {0, 0, 0, 0};



    /*
	int16_t xJoystickDirection = 0;
	int16_t yJoystickDirection = 0;
	int16_t rotationOfChassis = 0;
	*/

    // int8_t switcherForBuzzer = 0;
    // int16_t counterForBuzzer = 0;

    int16_t turretPitchNew = 4750;

    // int32_t actualTurretAngle = 0;


    double convertedAngle = 0;
    int16_t rotationSpeedOfChassis = 0;
    // int16_t 你是一个奴隶 = 5;

    int8_t chassisVsTurretDrive = 0;
    int32_t motorRotationCounter = 0;
	int64_t turretMotorPosition = 0;
	int16_t initialTurretPosition = getRotorPosition(Bus2, GM6020, 1);
	int16_t currentMotorPosition = getRotorPosition(Bus2, GM6020, 1);
	int16_t previousMotorPosition = getRotorPosition(Bus2, GM6020, 1);
	int16_t turingDirectionX = 0;
	int16_t turingDirectionY = 0;

	uint8_t blueX = 0;
	uint8_t blueY = 0;
	uint8_t redX = 0;
	uint8_t redY = 0;

	int8_t cantShoot = 0;



	// int8_t indexCounter = 0;
	// int32_t motorEncoderCounter = 0;
    //int16_t targetRPM[4] = {0,0,0,0};

  /* Infinite loop */
    for(;;) {

    	int8_t leftSwitch = getRCswitch(1);
    	int8_t rightSwitch = getRCswitch(0);

    	currentMotorPosition = getRotorPosition(Bus1, GM6020, 4);
    	if ((currentMotorPosition - previousMotorPosition) < -6000) {
    		motorRotationCounter++;
    	} else if ((currentMotorPosition - previousMotorPosition) > 6000) {
    		motorRotationCounter--;
    	}

    	turretMotorPosition = currentMotorPosition + (8191 * motorRotationCounter);

    	// actualTurretAngle = ((turretMotorPosition - initialTurretPosition)*0.025408f);
    	convertedAngle = ((turretMotorPosition - initialTurretPosition)* 0.0004434705f);


    	// usart_printf("%d %d %d\r\n", turretMotorPosition, initialTurretPosition, actualTurretAngle);
    	previousMotorPosition = getRotorPosition(Bus1, GM6020, 4);

	    for (int i = 0; i < 4; i++) {
	        rcRPM[i] = getRCchannel(i) * 13.645f;              // 13.645 = 469 / 187 / 660 * 3591, 660 = max reading in one direction
	    }

	    rcYaw = getRCchannel(4) * 0.85f;
	    chassisrcYaw = getRCchannel(0)*0.85f;


	    if (calibration == 1) {
	        motorRotationCounter = 0;
	    	turretMotorPosition = 0;
	    	initialTurretPosition = getRotorPosition(Bus1, GM6020, 1);
	        // actualTurretAngle = 0;
	        convertedAngle = 0;
	    }


	    int16_t turretPitch = getRCchannel(1)* 0.85f;

	    allCheShit(0, 0, 0, 0, rotationTime, rcRPM, 1, convertedAngle, chassisPreset);
	    if (shootingPhase == 1) {
			if(getBlueX() != 255) {
				blueX = getBlueX();
			}
			if(getBlueY() != 255) {
				blueY = getBlueX();
			}
			if(getRedX() != 255) {
				redX = getBlueX();
			}
			if(getRedY() != 255) {
				redY = getBlueX();
			}


			if ((firingStages == 1) && (robot_status.robot_id == 0)) {
				turingDirectionX = (-1*((int16_t)(getBlueX())-(setX)))*1;
				turingDirectionY = (-1*((int16_t)(getBlueY())-(setY)))*0.02;
				// usart_printf("working");
			} else if ((firingStages == 0) && (robot_status.robot_id == 0)){
				turingDirectionX = -1000;
				turingDirectionY = 0;
				turretPitchNew = 4750;
				// usart_printf("EROOR");
			}
			if ((firingStages == 1) && (robot_status.robot_id == 1)) {
				turingDirectionX = (-1*((int16_t)(getRedX())-(setX)))*1;
				turingDirectionY = (-1*((int16_t)(getRedY())-(setY)))*0.02;
			} else if ((firingStages == 0) && (robot_status.robot_id == 1)){
				turingDirectionX = -1000;
				turingDirectionY = 0;
				turretPitchNew = 4750;
			}
	    } else {
	    	turingDirectionY = 0;
	    	turingDirectionX = 0;
		    turretPitchNew -= turingDirectionY + turretPitch;
	    }
	    usart_printf("%d %d %d %d %d\r\n", getBlueX(), getBlueY(), turingDirectionX, turingDirectionY, firingStages);




	    if (turretPitchNew > 5170) {
	    	turretPitchNew = 5170;
	    } else if (turretPitchNew < 4330) {
	    	turretPitchNew = 4330;
	    }

	    setMotorPosition(Bus2, GM6020, 3, turretPitchNew, pitchPresetVoltagePosition);


	    //setMotorRPM(Bus2, M3508, 8, 50, shooterPreset);
	    float gyroVel[3] = {IMU_get_gyro(x), IMU_get_gyro(y), IMU_get_gyro(z)};

    	if (((gyroVel[2] > -0.02) && (gyroVel[2] < 0.02))) {
    		gyroVel[2] = 0;
    	}

    	gyroidValue += (gyroVel[2] + (0.008*((float)(turingDirectionX)))*0.4* mouseXScaler) + (0.008*((float)(getRCchannel(4)))*0.4*mouseXScaler);
    	setMotorRPM(Bus1, GM6020, 4, (int16_t)(10*gyroidValue), yawPresetVoltageRPM);

    	/*
	    if (chassisVsTurretDrive == 1) {
	    	// usart_printf("%f, %f, %f\r\n", gyroVel[0], gyroVel[1], gyroVel[2]);
	    	// gyroidValue += gyroVel[2]*0.005;
	    	if (((gyroVel[2] > -0.02) && (gyroVel[2] < 0.02))) {
	    		gyroVel[2] = 0;
	    	}

	    	gyroidValue += (gyroVel[2] + 0.008*((float)(pc_control.mouse_x)))*0.4* mouseXScaler;
	    	setMotorRPM(Bus1, GM6020, 1, (int16_t)(20*gyroidValue), yawPresetVoltageRPM);
	    } else {

	    	setMotorRPM(Bus1, GM6020, 1, -0.5*(int16_t)(mouseXScaler*(float)(pc_control.mouse_x)), yawPresetVoltageRPM);
	    }
		*/
    	uint16_t firingHeatTemp = power_heat_data.shooter_17mm_1_barrel_heat;


		if (firingHeatTemp > 350) {
			cantShoot = 1;
		} else {
			cantShoot = 0;
		}


	    int16_t flywheelSpeed = getMotorRPM(Bus2, M3508, 6);

	    if ((pc_control.right_button_down == 1) || (pc_control.left_button_down == 1) || (rightSwitch == 3) || (rightSwitch == 1) || (SHOOT == 1)) {
	    	motorOn = 1;
	    } else {
	    	motorOn = 0;
	    }


	    if ((pc_control.left_button_down == 1 || rightSwitch == 1 || SHOOT == 1) && (flywheelSpeed > 7000) && (cantShoot == 0)) {
	    	// setMotorRPM(Bus2, M2006, 6, (100), indexerPreset);


			if (jammed > 0 && indexerStopped == 0) {
				setMotorRPM(Bus2, M2006, 1, 15000, indexerPreset);
				jammed--;
			} else {
				indexerStopped = 0;
				setMotorRPM(Bus2, M2006, 1, -5400, indexerPreset);
				if (indexerTargetReached == 0 && getMotorRPM(Bus2, M2006, 1) <= -10) {
					indexerTargetReached = 1;
				} else if (getMotorRPM(Bus2, M2006, 1) > -1 && indexerTargetReached == 1) {         // jammed
					jammed = 12;
					indexerTargetReached = 0;
				}
			}
	    } else {
			indexerStopped = 1;
			indexerTargetReached = 0;
	    	setMotorRPM(Bus2, M2006, 1, 0, indexerPreset);
	    }

	    // PWMOff(Buzzer, 1);
		setMotorRPM(Bus2, M3508, 5, -7250*motorOn, flywheel);
		setMotorRPM(Bus2, M3508, 6, 7250*motorOn, flywheel);

		if (songPhase == 1) {
			uint32_t notes = HampsterNotes();
			PWMOutput(Buzzer, 1, notes);
			PWMInitialize(Buzzer, FR, 1, 0.5);
			PWMOn(Buzzer, 1);
		} else {
			PWMOff(Buzzer, 1);
		}



	    // int16_t motorSpeeddd = getMotorRPM(Bus1, GM6020, 1);

		PWMTimerStarter();
		RCkeysRefresh();
        osDelay(5);
    }
  /* USER CODE END TaskChassis */
}

/* USER CODE BEGIN Header_imu_temp_control_task */
/**
* @brief Function implementing the IMUtempPIDtask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_imu_temp_control_task */
__weak void imu_temp_control_task(void *argument)
{
  /* USER CODE BEGIN imu_temp_control_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END imu_temp_control_task */
}

/* USER CODE BEGIN Header_TaskTurret */
/**
* @brief Function implementing the turretTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TaskTurret */
void TaskTurret(void *argument)
{
  /* USER CODE BEGIN TaskTurret */
	// JoulesBuffer = 60;
  /* Infinite loop */
	for(;;)
	{
		osDelay(5);
	}
  /* USER CODE END TaskTurret */
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
