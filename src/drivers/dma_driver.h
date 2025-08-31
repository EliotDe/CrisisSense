#ifndef DMA_DRIVER_H
#define DMA_DRIVER_H

#include "stm32l432xx.h"
#include <stddef.h>

typedef enum{
    DMA_OK = 0,
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
}dma_peripheral_size_t;

// peripheral and memory increment mode -- mem-mem / periph-periph
typedef enum{
    DMA_PERIPH_INCREMENT_DISABLED,
    DMA_PERIPH_INCREMENT_ENABLED
}dma_peripheral_increment_mode_t;

typedef enum{
    DMA_MEMORY_INCREMENT_DISABLED,
    DMA_MEMORY_INCREMENT_ENABLED
}dma_memory_increment_mode_t;

// data transfer direction
typedef enum{
    DMA_READ_FROM_PERIPHERAL,
    DMA_READ_FROM_MEMORY
}dma_transfer_direction_t;

typedef uint32_t dma_irq_mask_t;
enum{ // bitwise enum
    DMA_INT_NONE = 0u,
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
    const void* periph_address;
    const void* memory_address;
    dma_memory_increment_mode_t mem_increment_mode;
    dma_peripheral_increment_mode_t periph_increment_mode;
    dma_transfer_direction_t transfer_direction;
    uint16_t data_size; // reference manual limits to 16 bits (0 to 2^16 - 1)
    dma_priority_t channel_priority;
    dma_memsize_t mem_size;
    dma_peripheral_size_t periph_size;
    dma_circular_mode_t circular_mode;
    dma_irq_mask_t interrupts;
}dma_channel_config_t;

/************* DMA CONFIGURATION FUNCTIONS *************/
uint8_t dma_config_channel(dma_channel_config_t* cfg, dma_err_t* error);
/********************* DMA HELPERS *********************/
static inline void dma_register_set_bit(uint32_t* reg, uint32_t bit){*reg |= bit;}
static inline void dma_register_clear_bit(uint32_t* reg, uint32_t bit){*reg &= ~bit;}


#endif