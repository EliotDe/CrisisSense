#include "dma_driver.h"
#include "stm32l432xx.h"

#define CHANNEL_SELECT_CODE_MAX 7u

static uint8_t dma_set_channel_request(const DMA_TypeDef* dma_line, uint8_t channel, uint8_t req, dma_err_t* error);

/*************  DMA CONFIGURATION CODE *************/

uint8_t dma_config_channel(const DMA_TypeDef* dma_line, dma_channel_config_t* cfg, dma_err_t* error){
  if(!cfg->dma_channel || !cfg->periph_address || !cfg->memory_address){
      if(error) *error = DMA_ERR_INVALID_PARAM;
      return 0;
  }
  if (cfg->channel_priority > DMA_PRIORITY_VERY_HIGH ||
      cfg->mem_increment_mode > DMA_MEMORY_INCREMENT_ENABLED ||
      cfg->periph_increment_mode > DMA_PERIPH_INCREMENT_ENABLED ||
      cfg->transfer_direction > DMA_READ_FROM_MEMORY ||
      cfg->mem_size > DMA_MSIZE_32 || 
      cfg->periph_size > DMA_PSIZE_32 ||
      cfg->circular_mode > DMA_CIRCULAR_ENABLED){
      if (error) *error = DMA_ERR_INVALID_PARAM;
      return 0;
  }

  const uint32_t IRQ_ALLOWED = (DMA_CCR_HTIE | DMA_CCR_TCIE | DMA_CCR_TEIE);

  if(cfg->interrupts & ~IRQ_ALLOWED){
      if(error) *error = DMA_ERR_INVALID_PARAM;
      return 0;
  }

  if(cfg->dma_channel->CCR & DMA_CCR_EN){
      if(error) *error = DMA_ERR_BUSY;
      return 0;
  }

  // set peripheral and memory address
  cfg->dma_channel->CPAR = (uint32_t)cfg->periph_address;
  cfg->dma_channel->CMAR = (uint32_t)cfg->memory_address;

  cfg->dma_channel->CNDTR = cfg->data_size;

  // Configure the channel request register
  dma_err_t channel_req_error = DMA_OK;
  dma_set_channel_request(dma_line, cfg->dma_channel_number, cfg->dma_channel_request, &channel_req_error);

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
      case DMA_PRIORITY_LOW: break; // Already handled by clearing bit
  }

  // Configure MSIZE
  dma_register_clear_bit(&ccr, DMA_CCR_MSIZE); // sets MSIZE[1:0] bits to 00 (DMA_MSIZE_8)
  if (cfg->mem_size == DMA_MSIZE_16)
      dma_register_set_bit(&ccr, DMA_CCR_MSIZE_0); // MSIZE[1:0] = 01
  else if (cfg->mem_size == DMA_MSIZE_32)
      dma_register_set_bit(&ccr, DMA_CCR_MSIZE_1); // MSIZE[1:0] = 10

  // Configure PSIZE
  dma_register_clear_bit(&ccr, DMA_CCR_PSIZE); // sets PSIZE[1:0] to 00
  if(cfg->periph_size == DMA_PSIZE_16)
      dma_register_set_bit(&ccr, DMA_CCR_PSIZE_0); // PSIZE[1:0] = 01
  else if (cfg->periph_size == DMA_PSIZE_32)
      dma_register_set_bit(&ccr, DMA_CCR_PSIZE_1); // PSIZE[1:0] = 10


  // Configure memory incremented mode
  if(cfg->mem_increment_mode == DMA_MEMORY_INCREMENT_DISABLED)
      dma_register_clear_bit(&ccr, DMA_CCR_MINC);
  else
      dma_register_set_bit(&ccr, DMA_CCR_MINC);

  // Configure peripheral incremented mode
  if(cfg->periph_increment_mode == DMA_PERIPH_INCREMENT_DISABLED)
      dma_register_clear_bit(&ccr, DMA_CCR_PINC);
  else 
      dma_register_set_bit(&ccr, DMA_CCR_PINC);

  // Configure Transfer Direction
  if(cfg->transfer_direction == DMA_READ_FROM_PERIPHERAL)
      dma_register_clear_bit(&ccr, DMA_CCR_DIR);
  else
      dma_register_set_bit(&ccr, DMA_CCR_DIR);
  
  // Configure Interrupts
  dma_register_clear_bit(&ccr, IRQ_ALLOWED);
  dma_register_set_bit(&ccr, cfg->interrupts & IRQ_ALLOWED);

  // set circular mode
  if (cfg->circular_mode == DMA_CIRCULAR_DISABLED)
      dma_register_clear_bit(&ccr, DMA_CCR_CIRC);
  else
      dma_register_set_bit(&ccr, DMA_CCR_CIRC);
  
  if(!(ccr & DMA_CCR_EN))
    ccr |= DMA_CCR_EN;

  cfg->dma_channel->CCR = ccr;

  
  return 1;
}

static uint8_t dma_set_channel_request(const DMA_TypeDef* dma_line, uint8_t channel, uint8_t req, dma_err_t* error){
  // if(!dma_channel){
  //   if(error) *error = DMA_ERR_INVALID_PARAM;
  //   return 0;
  // }
  if (channel > 7 ||
      req > CHANNEL_SELECT_CODE_MAX){
    if(error) *error = DMA_ERR_INVALID_PARAM;
    return 0;
  }
  uint32_t mask = 0xFU << ((channel - 1) * 4U);
  uint32_t val = ((uint32_t)(req & 0xFU)) << ((channel - 1) * 4U);

  //dma_channel->CCR &= ~DMA_CCR_EN; - channel should already be disabled when this is called
  if (dma_line == DMA1)
    DMA1_CSELR->CSELR = (DMA1_CSELR->CSELR & ~mask) | val;
  else 
    DMA2_CSELR->CSELR = (DMA2_CSELR->CSELR & ~mask) | val;
  return 1;
}

// Maybe an ISR that handles errors?