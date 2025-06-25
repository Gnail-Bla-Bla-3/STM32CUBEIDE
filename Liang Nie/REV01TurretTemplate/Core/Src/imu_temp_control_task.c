
#include "imu_temp_control_task.h"
#include "BMI088driver.h"
#include "BMI088Middleware.h"
#include "cmsis_os.h"
#include "main.h"
#include "pid.h"
#include "bsp_imu_pwm.h"
#include "UART.h"

#define IMU_temp_PWM(pwm)  imu_pwm_set(pwm)                    //pwm����

#define TEMPERATURE_PID_KP 1600.0f //kp of temperature control PID 
#define TEMPERATURE_PID_KI 0.2f    //ki of temperature control PID 
#define TEMPERATURE_PID_KD 0.0f    //kd of temperature control PID 

#define TEMPERATURE_PID_MAX_OUT 4500.0f  //max out of temperature control PID 
#define TEMPERATURE_PID_MAX_IOUT 4400.0f //max iout of temperature control PID 

extern GPIO_TypeDef* CS1_ACCEL_GPIO_Port;
extern uint16_t CS1_ACCEL_Pin;
extern GPIO_TypeDef* CS1_GYRO_GPIO_Port;
extern uint16_t CS1_GYRO_Pin;
extern SPI_HandleTypeDef hspi1;

extern uint16_t INT1_ACCEL_Pin;
extern uint16_t INT1_GRYO_Pin;

//task handler
TaskHandle_t INS_task_local_handler;


volatile uint8_t imu_start_flag = 0;

float PIDgyro[3], PIDaccel[3], PIDtemp;

//kp, ki,kd three params
const float imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
//pid struct 
pid_type_def imu_temp_pid;

/**
  * @brief          bmi088 temperature control 
  * @param[in]      argument: NULL
  * @retval         none
  */
/**
  * @brief          bmi088�¶ȿ���
  * @param[in]      argument: NULL
  * @retval         none
  */
void imu_temp_control_task(void const * argument)
{
    osDelay(500);
    //PID init
    PID_init(&imu_temp_pid, PID_POSITION, imu_temp_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT);

    //set spi frequency
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }
    //get task handle, must enable 'xTaskGetHandle' in cubeMX
    INS_task_local_handler = xTaskGetHandle(pcTaskGetName(NULL));
    imu_start_flag = 1;
    while(1)
    {

        //wait for task waked up
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS) {

        }
        uint16_t tempPWM;
        //calculate PID
        PID_calc(&imu_temp_pid, IMU_get_temp(), 40.0f);
        if (imu_temp_pid.out < 0.0f)
        {
            imu_temp_pid.out = 0.0f;
        }
        tempPWM = (uint16_t)imu_temp_pid.out;
        IMU_temp_PWM(tempPWM);
        osDelay(5);
    }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == INT1_ACCEL_Pin)
    {

        if(imu_start_flag)
        {
            //wake up the task
            if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
            {
                static BaseType_t xHigherPriorityTaskWoken;
                vTaskNotifyGiveFromISR(INS_task_local_handler, &xHigherPriorityTaskWoken);
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
        }
    }
    else if (GPIO_Pin == INT1_GRYO_Pin)
    {

    }
}
