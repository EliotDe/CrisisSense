#ifndef RCC_DRIVER_H
#define RCC_DRIVER_H

#include <stddef.h>

typedef enum{
  RCC_OK = 0,
  RCC_ERR_OUT_OF_RANGE = -1,
  RCC_ERR_HSE_NOT_AVAILABLE = -2,
}rcc_err_t;

size_t rcc_get_sysclk_hz();
size_t rcc_get_hclk_hz();
size_t rcc_get_pclk1_hz();
size_t rcc_get_pclk2_hz();

#endif