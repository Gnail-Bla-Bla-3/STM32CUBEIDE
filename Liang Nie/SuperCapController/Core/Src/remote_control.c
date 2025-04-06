/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       remote_control.c/h
  * @brief      ң��������ң������ͨ������SBUS��Э�鴫�䣬����DMA���䷽ʽ��ԼCPU
  *             ��Դ�����ô��ڿ����ж�������������ͬʱ�ṩһЩ��������DMA������
  *             �ķ�ʽ��֤�Ȳ�ε��ȶ��ԡ�
  * @note       ��������ͨ�������ж�����������freeRTOS����
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-01-2019     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "remote_control.h"
#include "UART.h"
#include "main.h"

extern pc_control_t pc_control;
extern UART_HandleTypeDef huart8;
extern DMA_HandleTypeDef hdma_usart8_rx;

uint8_t rx_buff[BUFF_SIZE];
extern RC_ctrl_t *local_rc_ctrl;

/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

//remote control data 
RC_ctrl_t rc_ctrl;
static RC_ctrl_t *rc_ptr = &rc_ctrl;


//receive data, 18 bytes one frame, but set 36 bytes 
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

/**
  * @brief          remote control init
  * @param[in]      none
  * @retval         none
  */
void remote_control_init(void) {
    RC_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}
/**
  * @brief          get remote control data point
  * @param[in]      none
  * @retval         remote control data point
  */
const RC_ctrl_t *get_remote_control_point(void) {
    return &rc_ctrl;
}
/*
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size) {
	usart_printf("UART Rx init = %d \r\n", 0);

	if(huart->Instance == UART8)
	{
		//usart_printf("UART Rx called = %d \r\n", 8);
		if (Size <= BUFF_SIZE)
		{
			HAL_UARTEx_ReceiveToIdle_DMA(&huart8, rx_buff, BUFF_SIZE*2);
			sbus_to_rc(rx_buff, local_rc_ctrl);
		}
		else
		{
			HAL_UARTEx_ReceiveToIdle_DMA(&huart8, rx_buff, BUFF_SIZE*2);
		}
	} else {
		//usart_printf("UART Rx called = %d \r\n", 0);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
	if(huart->Instance == UART8)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart8, rx_buff, BUFF_SIZE*2);
	}
}

void HAL_UART_Rx(UART_HandleTypeDef * huart)
{
	//HAL_UARTEx_ReceiveToIdle_DMA(huart, rx_buff, BUFF_SIZE*2);
}*/

/*
void USART8_IRQHandler(void) {
    if(huart8.Instance->SR & UART_FLAG_RXNE) {
        __HAL_UART_CLEAR_PEFLAG(&huart8);
    }
    else if(USART8->SR & UART_FLAG_IDLE) {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart8);

        if ((hdma_usart8_rx.Instance->CR & DMA_SxCR_CT) == RESET) {
            // Current memory buffer used is Memory 0
    
            //disable DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart8_rx.Instance->NDTR;

            //reset set_data_lenght
            hdma_usart8_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            hdma_usart8_rx.Instance->CR |= DMA_SxCR_CT;

            //enable DMA
            __HAL_DMA_ENABLE(&hdma_usart8_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH) {
                sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
            }
        } else {
            // Current memory buffer used is Memory 1
            //disable DMA
            __HAL_DMA_DISABLE(&hdma_usart8_rx);

            //get receive data length, length = set_data_length - remain_length
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart8_rx.Instance->NDTR;

            //reset set_data_length
            hdma_usart8_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            DMA2_Stream0->CR &= ~(DMA_SxCR_CT);

            //enable DMA
            __HAL_DMA_ENABLE(&hdma_usart8_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
            }
        }
    }
}*/

void usart_printRC(void) {
	usart_printf(
	"\r\n\
	ch0:%d | ch1:%d | ch2:%d | ch3:%d | ch4:%d\r\n\
	s1:%d | s2:%d | mouse_x:%d | mouse_y:%d\r\n\
	m1:%d | m2:%d | key:%d\r\n\
	\r\n",
	rc_ptr->rc.ch[0], rc_ptr->rc.ch[1], rc_ptr->rc.ch[2], rc_ptr->rc.ch[3], rc_ptr->rc.ch[4],
	rc_ptr->rc.s[0], rc_ptr->rc.s[1], rc_ptr->mouse.x, rc_ptr->mouse.y,rc_ptr->mouse.z,
	rc_ptr->mouse.press_l, rc_ptr->mouse.press_r, rc_ptr->key.v);
}

/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl) {
    if (sbus_buf == NULL || rc_ctrl == NULL) {
        return;
    }

    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) &0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Pressed ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Pressed ?
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
}

int16_t getRCchannel(int8_t channelID) {
	return rc_ptr->rc.ch[channelID];
}

int8_t getRCswitch(int8_t switchID) {
	return rc_ptr->rc.s[switchID];
}

int16_t getRCmouseMovement(int8_t axis) {
	if(axis == 0){
		return rc_ptr->mouse.x;
	} else if (axis == 1) {
		return rc_ptr->mouse.y;
	} else if (axis == 2) {
		return rc_ptr->mouse.z;
	} else {
		return 0;
	}
}

void RCkeysRefresh(void) {//temporary until uart fixed
	uint16_t key = rc_ptr->key.v;
	if(key > 32767){
		pc_control.b = 1;
		key= key - 32768;
	}else{
		pc_control.b = 0;
	}
	if(key>16383){
		pc_control.v = 1;
		key= key - 16384;
	}else{
		pc_control.v = 0;
	}
	if(key>8191){
		pc_control.c = 1;
		key= key - 8192;
	}else{
		pc_control.c = 0;
	}
	if(key>4095){
		pc_control.x = 1;
		key= key - 4096;
	}else{
		pc_control.x = 0;
	}
	if(key>2047){
		pc_control.z = 1;
		key= key - 2048;
	}else{
		pc_control.z = 0;
	}
	if(key>1023){
		pc_control.g = 1;
		key= key - 1024;
	}else{
		pc_control.g = 0;
	}
	if(key>511){
		pc_control.f = 1;
		key= key - 512;
	}else{
		pc_control.f = 0;
	}
	if(key > 255){
		pc_control.r = 1;
		key = key - 256;
	}else{
		pc_control.r = 0;
	}


	if(key > 127){
		pc_control.e = 1;
		key = key - 128;
	}else{
		pc_control.e = 0;

	}
	if(key>63){
		pc_control.q = 1;
		key= key - 64;
	}else{
		pc_control.q = 0;
	}
	if(key>31){
		pc_control.ctrl = 1;
		key =key - 32;
	}else{
		pc_control.ctrl = 0;
	}
	if(key>15){
		pc_control.shift = 1;
		key= key - 16;
	}else{
		pc_control.shift = 0;
	}
	if(key>7){
		pc_control.d = 1;
		key= key - 8;
	}else{
		pc_control.d = 0;
	}
	if(key>3){
		pc_control.a = 1;
		key= key - 4;
	}else{
		pc_control.a = 0;
	}
	if(key>1){
		pc_control.s = 1;
		key= key - 2;
	}else{
		pc_control.s = 0;
	}
	if(key > 0){
		pc_control.w = 1;
	}else{
		pc_control.w = 0;
	}

	pc_control.mouse_x = rc_ptr->mouse.x;
	pc_control.mouse_y = rc_ptr->mouse.y;
	pc_control.mouse_z = rc_ptr->mouse.z;

	pc_control.left_button_down = rc_ptr->mouse.press_l;
	pc_control.right_button_down = rc_ptr->mouse.press_r;
}
