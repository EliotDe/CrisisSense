#ifndef DEBUGGING_MANAGER_H
#define DEBUGGING_MANAGER_H

#include "stm32l432xx.h"
#include <stddef.h>



uint8_t Manager_Debug_Polling(const char* debug_msg);
uint8_t Manager_Debug_DMA(const char* mem_address);

#endif