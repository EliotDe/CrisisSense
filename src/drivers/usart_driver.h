#ifndef USART_DRIVER_H
#define USART_DRIVER_H

#ifdef UNIT_TEST
    #include "tests\mock_stm32l432xx.h"
#else
    #include "stm32l432xx.h"
#endif //UNIT_TEST

#include <stddef.h>

// Error Codes
typedef enum{
    USART_OK = 0,
    USART_ERR_INVALID_PARAM = -1,
    USART_ERR_BUSY = -2,
    USART_ERR_OVERFLOW = -3

} usart_err_t;

typedef enum{
    USART_MODE_POLLING,
    USART_MODE_INTERRUPTS,
    USART_MODE_DMA
} usart_mode_t;

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

typedef enum{
    USART_DMA_NONE, 
    USART_DMA_RX = USART_CR3_DMAR,   // Use DMA on received data
    USART_DMA_TX = USART_CR3_DMAT,    // Use DMA on transmitted data
    USART_DMA_RX_TX = (USART_CR3_DMAR | USART_CR3_DMAT)
}usart_dma_t;

// DMA Disable on Recetpion Error
typedef enum{
    USART_DMA_NOT_DISABLED_ON_ERROR,
    USART_DMA_DISABLED_ON_ERROR
}usart_dma_ddre_t;

typedef enum{
    USART_AUTOBAUD_DISABLED,
    USART_AUTOBAUD_START_BIT,
    USART_AUTOBAUD_FALLING_EDGE = USART_CR2_ABRMODE_0,
    USART_AUTOBAUD_7F_FRAME = USART_CR2_ABRMODE_1,
    USART_AUTOBAUD_55_FRAME = (USART_CR2_ABRMODE_0 | USART_CR2_ABRMODE_1)
}usart_autobaud_t; 

typedef struct{
    uint32_t baud_rate;
    usart_oversampling_t oversampling;
    uint32_t f_clk;
} usart_baud_config_t;

typedef struct{
    const void* buffer;
    size_t length;
    usart_word_length_t word_length;
    usart_mode_t mode;
} usart_dataPacket_t;

typedef struct{
    USART_TypeDef* usart_line;
    usart_parity_t parity;
    usart_autobaud_t auto_baud;
    uint32_t baud_rate;
    uint32_t f_clk;
    usart_oversampling_t oversampling;
    usart_word_length_t word_length;
    usart_stop_bits_t stop_bits;
    usart_dma_t dma;
    usart_dma_ddre_t dma_ddre;
} usart_config_t;

/*=============CONFIGURATION FUNCTIONS=============*/

uint8_t usart_config_line(usart_config_t* cfg, usart_err_t* error);
uint8_t usart_transmit(USART_TypeDef* usart_line, const usart_dataPacket_t* data_packet, usart_err_t* error);

/*==============CONFIGURATION HELPERS==============*/

static inline void usart_register_set_bit(uint32_t* reg, uint32_t bit){*reg |= bit;}
static inline void usart_register_clear_bit(uint32_t* reg, uint32_t bit){*reg &= ~bit;}



#endif //USART_DRIVER_H