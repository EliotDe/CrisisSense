#ifndef DMA_DRIVER_H
#define DMA_DRIVER_H

#include "stm32l432xx.h"
#include <stddef.h>

typedef enum{
    DMA_ERR_INVALID_PARAM = -1,
    DMA_ERR_BUSY = -2,
}dma_err_t;

/**
     * configure in CCR:
     * – the channel priority
     * – the data transfer direction
     * – the circular mode
     * – the peripheral and memory incremented mode
     * – the peripheral and memory data size
     * – the interrupt enable at half and/or full transfer and/or transfer error
     */

typedef enum{
    DMA_PRIORITY_LOW,
    DMA_PRIORITY_MEDIUM,
    DMA_PRIORITY_HIGH,
    DMA_PRIORITY_VERY_HIGH
}dma_priority_t;

typedef enum{
    DMA_MSIZE_8,
    DMA_MSIZE_16,
    DMA_MSIZE_32
}dma_memsize_t;

typedef enum{
    DMA_PSIZE_8,
    DMA_PSIZE_16,
    DMA_PSIZE_32
}dma_periphsize_t;

// peripheral and memory increment mode -- mem-mem / periph-periph
// data transfer direction -- mem-mem / periph-periph

typedef enum{ // bitwise enum
    DMA_INT_HALF_TRANSFER = DMA_CCR_HTIE,
    DMA_INT_TRANSFER_COMPLETE = DMA_CCR_TCIE,
    DMA_INT_TRANSFER_ERROR = DMA_CCR_TEIE
}dma_interrupts_t;

typedef enum{
    DMA_CIRCULAR_DISABLED,
    DMA_CIRCULAR_ENABLED
}dma_circular_mode_t;

typedef struct{
    DMA_Channel_TypeDef* dma_channel;
    uint32_t* periph_address;
    uint32_t* memory_address;
    uint16_t data_size; // reference manual limits to 16 bits (0 to 2^16 - 1)
    dma_priority_t channel_priority;
    dma_memsize_t mem_size;
    dma_periphsize_t periph_size;
    dma_circular_mode_t circular_mode;
    dma_interrupts_t interrupts;
}dma_channel_config_t;

/************* DMA CONFIGURATION FUNCTIONS *************/
uint8_t dma_config_channel(DMA_TypeDef* dma_line, dma_channel_config_t* cfg, dma_err_t* error);
/********************* DMA HELPERS *********************/
static inline void dma_register_set_bit(uint32_t* reg, uint32_t bit){*reg |= bit;}
static inline void dma_register_clear_bit(uint32_t* reg, uint32_t bit){*reg &= ~bit;}


#endif