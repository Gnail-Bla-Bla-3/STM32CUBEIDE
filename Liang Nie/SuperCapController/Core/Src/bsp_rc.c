#include "bsp_rc.h"
#include "main.h"

extern UART_HandleTypeDef huart8;
extern DMA_HandleTypeDef hdma_uart8_rx;

void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num) {

    SET_BIT(huart8.Instance->CR3, USART_CR3_DMAR);                 //enable the DMA transfer for the receiver request
    __HAL_UART_ENABLE_IT(&huart8, UART_IT_IDLE);                  //enalbe idle interrupt
    __HAL_DMA_DISABLE(&hdma_uart8_rx);                   //disable DMA
    while(hdma_uart8_rx.DMAmuxChannelStatus == HAL_DMA_STATE_RESET) {
        __HAL_DMA_DISABLE(&hdma_uart8_rx);
    }
    //hdma_uart8_rx.Instance->PAR = (uint32_t) & (USART8->DR);                   //memory buffer 1
    //hdma_uart8_rx.Instance->M0AR = (uint32_t)(rx1_buf);                 //memory buffer 2
    //hdma_uart8_rx.Instance->M1AR = (uint32_t)(rx2_buf);                  //data length
    //hdma_uart8_rx.Instance->NDTR = dma_buf_num;                 //enable double memory buffer
    //SET_BIT(hdma_uart8_rx.Instance->CR, DMA_SxCR_DBM);
    __HAL_DMA_ENABLE(&hdma_uart8_rx);                 //enable DMA

}






