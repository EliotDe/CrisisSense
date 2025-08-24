#ifndef USART_DRIVER_H
#define USART_DRIVER_H

#include "stm32l432xx.h"

// Error Codes
typedef enum{
    USART_OK = 0,
    USART_ERR_INVALID_PARAM = -1,
    USART_ERR_BUSY = -2,
    USART_ERR_OVERFLOW = -3

} usart_err_t;

// Parity Setting
typedef enum{
    USART_PARITY_NONE,
    USART_PARITY_EVEN,
    USART_PARITY_ODD
} usart_parity_t;

// Word Length Setting
typedef enum{
    USART_WORD_LENGTH_8,
    USART_WORD_LENGTH_9,
    USART_WORD_LENGTH_7
} usart_word_length_t;

// Oversampling by 16 or by 8
typedef enum{
    USART_OVER16,
    USART_OVER8
}usart_oversampling_t;

// Stop Bits
typedef enum{
    USART_STOP_1 = 0,
    USART_STOP_HALF = 1,
    USART_STOP_2 = 2,
    USART_STOP_1_HALF = 3
}usart_stop_bits_t;

typedef struct{
    USART_TypeDef* usart_line;
    usart_parity_t parity;
    uint32_t baud_rate;
    usart_oversampling_t oversampling;
    usart_word_length_t word_length;
    usart_stop_bits_t stop_bits;
} usart_config_t;

/*=============CONFIGURATION FUNCTIONS=============*/

uint8_t usart_config_line(usart_config_t* cfg, usart_err_t* error);
uint8_t usart_config_baudrate(USART_TypeDef* usart_line, uint32_t baud_rate, usart_oversampling_t over8, usart_err_t* error);
uint8_t usart_config_dma(USART_TypeDef* usart_line, usart_err_t* error);
/*==============CONFIGURATION HELPERS==============*/

static inline void usart_register_set_bit(uint32_t* reg, uint32_t bit){*reg |= bit;}
static inline void usart_register_clear_bit(uint32_t* reg, uint32_t bit){*reg &= ~bit;}



#endif