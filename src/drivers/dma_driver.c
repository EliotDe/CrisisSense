#include "dma_driver.h"
#include "stm32l432xx.h"

/*************  DMA CONFIGURATION CODE *************/

uint8_t dma_config_channel(DMA_TypeDef* dma_line, dma_channel_config_t* cfg, dma_err_t* error){
    if(!dma_line || !cfg->dma_channel){
        if(error) *error = DMA_ERR_INVALID_PARAM;
        return 0;
    }
    if (cfg->channel_priority > DMA_PRIORITY_VERY_HIGH ||
        cfg->mem_size > DMA_MSIZE_32 || 
        cfg->periph_size > DMA_PSIZE_32 ||
        cfg->circular_mode > DMA_CIRCULAR_ENABLED ||
        cfg->interrupts > (DMA_INT_HALF_TRANSFER | DMA_INT_TRANSFER_COMPLETE | DMA_INT_TRANSFER_ERROR)){
        if (error) *error = DMA_ERR_INVALID_PARAM;
        return 0;
    }
    if(cfg->dma_channel->CCR & DMA_CCR_EN){
        if(error) *error = DMA_ERR_BUSY;
        return 0;
    }

    // set peripheral and memory address
    cfg->dma_channel->CPAR = cfg->periph_address;
    cfg->dma_channel->CMAR = cfg->memory_address;

    cfg->dma_channel->CNDTR = cfg->data_size;

    /**
     * configure in CCR:
     * – the channel priority
     * – the data transfer direction
     * – the circular mode
     * – the peripheral and memory incremented mode
     * – the peripheral and memory data size
     * – the interrupt enable at half and/or full transfer and/or transfer error
    */

    uint32_t ccr = cfg->dma_channel->CCR;
    
    dma_register_clear_bit(&ccr, DMA_CCR_PL); // this function sets PL[1:0] to 00 (DMA_PRIORITY_LOW)
    switch(cfg->channel_priority){
        case DMA_PRIORITY_MEDIUM: dma_register_set_bit(&ccr, DMA_CCR_PL_0); break; // set PL bits to 01 (header definitions are a bit weird)
        case DMA_PRIORITY_HIGH: dma_register_set_bit(&ccr, DMA_CCR_PL_1); break; // set PL bits to 10 
        case DMA_PRIORITY_VERY_HIGH: dma_register_set_bit(&ccr, DMA_CCR_PL); break; // sets PL bits to 11
    }

    // Configure MSIZE
    dma_register_clear_bit(&ccr, DMA_CCR_MSIZE); // sets MSIZE[1:0] bits to 00 (DMA_MSIZE_8)
    if (cfg->mem_size == DMA_MSIZE_16)
        dma_register_set_bit(&ccr, DMA_CCR_MSIZE_0); // MSIZE[1:0] = 01
    else
        dma_register_set_bit(&ccr, DMA_CCR_MSIZE_1); // MSIZE[1:0] = 10

    // Configure PSIZE
    dma_register_set_bit(&ccr, DMA_CCR_PSIZE); // sets PSIZE[1:0] to 00
    if(cfg->periph_size == DMA_PSIZE_16)
        dma_register_set_bit(&ccr, DMA_CCR_PSIZE_0); // PSIZE[1:0] = 01
    else
        dma_register_set_bit(&ccr, DMA_CCR_PSIZE_1); // PSIZE[1:0] = 10

    // Configure incremented mode  &  data transfer direction
    
    // Configure Interrupts
    dma_register_set_bit(&ccr, cfg->interrupts);

    // set circular mode
    if (cfg->circular_mode == DMA_CIRCULAR_DISABLED)
        dma_register_clear_bit(&ccr, DMA_CCR_CIRC);
    else
        dma_register_set_bit(&ccr, DMA_CCR_CIRC);
    
    
    cfg->dma_channel->CCR = ccr;
    
    return 1;
}


// Maybe an ISR that handles errors?