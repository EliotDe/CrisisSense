#ifndef DEBUGGING_MANAGER_H
#define DEBUGGING_MANAGER_H

#include "stm32l432xx.h"
#include <stddef.h>
#include "usart_driver.h"


typedef enum{
    DEBUG_OK,
    ERR_DEBUG_PCLK_NOT_RECEIVED = -1,
}debug_err_t;

uint8_t Manager_Debug_Polling(const char* debug_msg);
uint8_t Manager_Debug_DMA(const char* mem_address);
int8_t Manager_USART_Config(usart_mode_t usart_mode);
uint8_t Manager_GPIO_Config(void);
uint8_t Manager_RCC_Config(void);

#endif