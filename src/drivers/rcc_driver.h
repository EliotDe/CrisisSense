#ifndef RCC_DRIVER_H
#define RCC_DRIVER_H

#include <stddef.h>

typedef enum{
  RCC_OK = 0,
  RCC_ERR_OUT_OF_RANGE = -1,
  RCC_ERR_HSE_NOT_AVAILABLE = -2,
  RCC_ERR_MSI_NOT_READY = -3,
  RCC_ERR_INVALID_PLL_SRC = -4,
  RCC_ERR_INVALID_SYSCLK_SRC = -5
}rcc_err_t;

// Removed to appease cppcheck
// uint32_t  rcc_get_sysclk_hz();
// uint32_t  rcc_get_hclk_hz();
uint32_t  rcc_get_pclk1_hz();
uint32_t  rcc_get_pclk2_hz();

#endif