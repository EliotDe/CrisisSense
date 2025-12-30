/**
 * @brief This Is the SPI Driver
 * @todo Add RXONLY-specific disable procedure functionality
 * @todo Consider enforcing mem-alignment for datasizes greater than 8-bits - STM32 h/ware enforces this, but just to be safe consider it.
 */

#include "stm32l432xx.h"
#include "spi_driver.h"
#include <stddef.h>


#define SPI_DATA_SIZE_MIN 4u
#define SPI_DATA_SIZE_CHAR 8u
#define SPI_DATA_SIZE_MAX 16u

#define SPI_RXNE_TIMEOUT 10000u
#define SPI_TXE_TIMEOUT 50000u

#define SPI_INSTANCE_COUNT 2u

/**
 * Enumerates the SPI peripheral state.
 * States Include:
 * - SPI_STATE_IDLE: Peripheral is not being used
 * - SPI_STATE_ACTIVE: Peripheral is enabled
 * - SPI_STATE_SHUTDOWN_PENDING: Peripheral is pending shutdown
 * - SPI_STATE_SHUTDOWN_TX_EMPTIED: FTLVL[1:0] = 00, wait for BSY = 0
 * -  
 */
typedef enum{
  SPI_STATE_IDLE,
  SPI_STATE_ACTIVE,                 
  SPI_STATE_SHUTDOWN_PENDING,       // Transition out of this state once TXFIFO is empty
  SPI_STATE_SHUTDOWN_TX_EMPTIED,    // Transition out of this state once BSY = 0
  SPI_STATE_SHUTDOWN_BSY_0,         // Transition out of this state once the RXFIFO register is emptied
  SPI_STATE_SHUTDOWN_RX_EMPTIED     // Transition out of this state when disable procedure is complete
}spi_state_t;

typedef struct{
  SPI_TypeDef* spi;
  volatile spi_state_t state;
  uint8_t rxonly;
  uint8_t using_dma;
  uint8_t tx_in_progress;
  //uint32_t deadline_ms; // Timeout (non-blocking) -- optional
  const void* tx_buffer;
  void* rx_buffer;
  uint32_t tx_buffer_size;
  uint32_t rx_buffer_size;
  volatile uint32_t tx_index;
  volatile uint32_t rx_index;
  //uint32_t expected_rx_frames;
  //uint32_t rx_count;
}spi_context_t;


static spi_context_t spi_states[SPI_INSTANCE_COUNT] = {// For SPI1 and SPI3 -- Should it be Volatile?
  {.spi = SPI1, .state = SPI_STATE_IDLE}, //SPI1 - initial context
  {.spi = SPI3, .state = SPI_STATE_IDLE}  //SPI3 - initial context
}; 

static inline spi_context_t* spi_get_context(const SPI_TypeDef* spi_line);
static int8_t spi_check_busy_and_disable(SPI_TypeDef* spi_line, spi_context_t* ctx);
static int8_t spi_handle_txe(SPI_TypeDef* spi_line, spi_context_t* ctx, uint8_t data_size);
static int8_t spi_handle_rxne(SPI_TypeDef* spi_line, spi_context_t* ctx, uint8_t data_size);
static int8_t SPIx_IRQHandler(SPI_TypeDef* spi_line, spi_context_t* ctx, uint8_t data_size);

/*============================ SPI HELPER FUNCTIONS ======================================*/

/**
 * @brief This helper function inputs the spi peripheral and returns the pointer to its context struct
 * @param spi_line The SPI peripheral 
 * @retval The pointer to the SPI peripherals context struct
 * @retval NULL - if an Invalid spi_line is passed, e.g. SPI1276 
 */
static inline spi_context_t* spi_get_context(const SPI_TypeDef* spi_line){
  if(spi_line == SPI1) return &spi_states[0];
  if(spi_line == SPI3) return &spi_states[1];
  return NULL; // Invalid spi_line passed
}

/**
 * @brief Blocking wait function. 
 * @param reg Pointer to a 32-bit register
 * @param mask The bit to wait for, e.g. SPI_CR1_SPE
 * @param timeout How long to wait for the bit before returning a timeout error
 * 
 * @retval SPI_OK if the wait is succesful, SPI_ERR_TIMEOUT if it times out
 */
static inline int8_t wait_until_clear(volatile const uint32_t* reg, uint32_t mask, uint32_t timeout){
  while((*reg & mask) != 0U){
    if(timeout == 0) return SPI_ERR_TIMEOUT;
    timeout--;
  }
  return SPI_OK;
}

/**
 * @brief Mask the NVIC line on a given SPI peripheral
 * @param spi_line The SPI peripheral to disable interrupts on
 * 
 * @note Blocks all IRQ sources (TXE,RXNE,ERR,DMA requests)
 * @note Might be a little heavy handed to disable all interrupts in most cases - Use something else if necessary
 */
static inline void spi_mask_line_irq(const SPI_TypeDef* spi_line){
  if(spi_line == SPI1)
    __NVIC_DisableIRQ(SPI1_IRQn);
  else if (spi_line == SPI3)
    __NVIC_DisableIRQ(SPI3_IRQn);
}

/**
 * @brief Enable interrupts on a given SPI peripheral
 * @param spi_line The SPI peripheral to enable interrupts on
 */
static inline void spi_unmask_line_irq(const SPI_TypeDef* spi_line){
  if(spi_line == SPI1)
    __NVIC_EnableIRQ(SPI1_IRQn);
  else if (spi_line == SPI3)
    __NVIC_EnableIRQ(SPI3_IRQn);
}

/**
 * @brief Checks whether interrupts are enabled on the given SPI peripheral
 * @param spi_line The SPI peripheral
 * @retval 0 IRQ is not enabled, 1 IRQ is enabled, <0 Error code SPI_ERR_X
 */
static inline int8_t spi_get_irq_status(const SPI_TypeDef* spi_line){
  if(spi_line == SPI1)
    return __NVIC_GetEnableIRQ(SPI1_IRQn);
  else if(spi_line == SPI3)
    return __NVIC_GetEnableIRQ(SPI3_IRQn);

  return SPI_ERR_INVALID_PARAM;
}
/*=========================== SPI CONFIGURATION CODE ================================*/



/**
 * @brief Configure the SPI peripheral
 * @param spi_line Pointer to the SPI peripheral to be configured
 * @param cfg Pointer to the configuration struct containing settings for the spi peripheral
 * 
 * @retval An Error Code of Format SPI_ERR_X or SPI_OK (if there was no error)
 * @note Doesn't Enable SPI
 */
int8_t spi_config(SPI_TypeDef* spi_line, const spi_config_t* cfg){
  if(!spi_line || !cfg)
    return SPI_ERR_INVALID_PARAM;

  if(spi_line->CR1 & SPI_CR1_SPE)
    return SPI_ERR_BUSY;
  
  // Range Validation
  if(cfg->baud_rate_factor > SPI_BAUD_DIV256 ||
     cfg->bidi_direction > SPI_BIDI_TRANSMIT_ONLY ||
     cfg->bidirectional_mode > SPI_BIDIRECTIONAL ||
     cfg->simplex_mode > SPI_RECEIVE_ONLY ||
     cfg->clock_phase > SPI_CLK_PHASE1 ||
     cfg->clock_polarity > SPI_CLK_IDLE1 ||
     cfg->crc_code_length > SPI_CRC_LENGTH_16 ||
     cfg->dma_rx_data_length > SPI_DMA_RX_ODD_DATA ||
     cfg->dma_tx_data_length > SPI_DMA_TX_ODD_DATA ||
     cfg->enable_crc > SPI_CRC_ENABLED ||
     cfg->fifo_rx_threshold > SPI_FIFO_LVL_QUARTER ||
     cfg->frame_format > SPI_TI_MODE ||
     cfg->lsb_first > SPI_LSB_FIRST ||
     cfg->master_selction > SPI_MASTER_CONFIG ||
     cfg->nss_pulse > SPI_GENERATE_NSS_PULSE ||
     cfg->slave_select_enabled > SPI_SS_OUTPUT_ENABLED ||
     cfg->ssi_bit > SPI_SSI_HIGH || 
     cfg->ssm_enabled > SPI_SSM_ENABLED ||
     cfg->data_size_in_bits < SPI_DATA_SIZE_MIN ||
     cfg->data_size_in_bits > SPI_DATA_SIZE_MAX){
    return SPI_ERR_INVALID_PARAM;
  }

  // Bidirectional & Receive-Only bits cannot be set at the same time
  if(cfg->bidirectional_mode == SPI_BIDIRECTIONAL &&
     cfg->simplex_mode == SPI_RECEIVE_ONLY)
    return SPI_ERR_INVALID_PARAM;


  // Configure SPI_CR1 register
  //uint32_t cr1 = spi_line->CR1;
  
  // CURRENTLY DOESN'T HANDLE ERRORS RECEIVED FROM WRITE_BITS
  write_bits(&spi_line->CR1, (uint32_t)SPI_CR1_BR_Msk, (uint32_t)SPI_CR1_BR_Pos, cfg->baud_rate_factor);
  write_bits(&spi_line->CR1, (uint32_t)SPI_CR1_CPOL_Msk, (uint32_t)SPI_CR1_CPOL_Pos, cfg->clock_polarity);
  write_bits(&spi_line->CR1, (uint32_t)SPI_CR1_CPHA_Msk, (uint32_t)SPI_CR1_CPHA_Pos, cfg->clock_phase);
  write_bits(&spi_line->CR1, (uint32_t)SPI_CR1_BIDIMODE_Msk, (uint32_t)SPI_CR1_BIDIMODE_Pos, cfg->bidirectional_mode); 
  write_bits(&spi_line->CR1, (uint32_t)SPI_CR1_BIDIOE_Msk, (uint32_t)SPI_CR1_BIDIOE_Pos, cfg->bidi_direction);
  write_bits(&spi_line->CR1, (uint32_t)SPI_CR1_RXONLY_Msk, (uint32_t)SPI_CR1_RXONLY_Pos, cfg->simplex_mode);
  write_bits(&spi_line->CR1, (uint32_t)SPI_CR1_LSBFIRST_Msk, (uint32_t)SPI_CR1_LSBFIRST_Pos, cfg->lsb_first);
  write_bits(&spi_line->CR1, (uint32_t)SPI_CR1_CRCL_Msk, (uint32_t)SPI_CR1_CRCL_Pos, cfg->crc_code_length);
  write_bits(&spi_line->CR1, (uint32_t)SPI_CR1_CRCEN_Msk, (uint32_t)SPI_CR1_CRCEN_Pos, cfg->enable_crc);
  write_bits(&spi_line->CR1, (uint32_t)SPI_CR1_SSM_Msk, (uint32_t)SPI_CR1_SSM_Pos, cfg->ssm_enabled);
  write_bits(&spi_line->CR1, (uint32_t)SPI_CR1_SSI_Msk, (uint32_t)SPI_CR1_SSI_Pos, cfg->ssi_bit);
  write_bits(&spi_line->CR1, (uint32_t)SPI_CR1_MSTR_Msk, (uint32_t)SPI_CR1_MSTR_Pos, cfg->master_selction);
  
  //spi_line->CR1 = cr1;

  // Configure SPI_CR2 register
  //uint32_t cr2 = spi_line->CR2;

  uint8_t data_size = cfg->data_size_in_bits;
  uint8_t data_size_bits = data_size - 1;       // DS[3:0] is stored like this (0011: 4-bit, 0100: 5-bit, ..., 1111: 16-bit)
  write_bits(&spi_line->CR2, (uint32_t)SPI_CR2_DS_Msk, (uint32_t)SPI_CR2_DS_Pos, data_size_bits);
  write_bits(&spi_line->CR2, (uint32_t)SPI_CR2_SSOE_Msk, (uint32_t)SPI_CR2_SSOE_Pos, cfg->slave_select_enabled);
  write_bits(&spi_line->CR2, (uint32_t)SPI_CR2_FRF_Msk, (uint32_t)SPI_CR2_FRF_Pos, cfg->frame_format);
  write_bits(&spi_line->CR2, (uint32_t)SPI_CR2_NSSP_Msk, (uint32_t)SPI_CR2_NSSP_Pos, cfg->nss_pulse);
  write_bits(&spi_line->CR2, (uint32_t)SPI_CR2_FRXTH_Msk, (uint32_t)SPI_CR2_FRXTH_Pos, cfg->fifo_rx_threshold);
  write_bits(&spi_line->CR2, (uint32_t)SPI_CR2_LDMATX_Msk, (uint32_t)SPI_CR2_LDMATX_Pos, cfg->dma_tx_data_length);
  write_bits(&spi_line->CR2, (uint32_t)SPI_CR2_LDMARX_Msk, (uint32_t)SPI_CR2_LDMARX_Pos, cfg->dma_rx_data_length);

  //spi_line->CR2 = cr2;
  
  spi_line->CRCPR = cfg->crc_polynomial;

  return SPI_OK;
}

/**
 * @brief This function starts the transmission process on an SPI peripheral using for DMA transfer.
 * @param spi_line SPI peripheral to transmit on
 * 
 * @note SPI must be configured (via spi_config(:|))  
 * @retval An error code of form SPI_ERR_X, or SPI_OK if no error occured
 * 
 * @todo If an interrupt manipulates CR1, write may overwrite changes
 * @todo consider using a critical section
 */
int8_t spi_enable(SPI_TypeDef* spi_line){
  // Validation
  if(!spi_line)
    return SPI_ERR_INVALID_PARAM;

  if(spi_line->CR1 & SPI_CR1_SPE)
    return SPI_ERR_BUSY;

  // Atomic Write to CR1 - if an interrupt occurs between this could overwrite changes to CR1 (not sure whether i should fix this, im not sure an interrupt would occur here)
  spi_line->CR1 |= SPI_CR1_SPE;
  __DSB();
  __ISB();


  return SPI_OK;
}

/**
 * @brief This function executes the SPI disabling procedure, outlined in the reference manual and necessary to avoid corrupted transmissions.
 * @param spi_line The line to disable
 * @param rxonly If the SPI peripheral is in a read only mode, the disable procedure is different
 * 
 * @todo Switch to interrupt based
 * @retval An error code of form SPI_ERR_X, or SPI_OK if no error occured
 */
int8_t spi_disable(SPI_TypeDef* spi_line, spi_rxonly_t rxonly){
  /**
   * When the master is in any receive only mode, the only way to stop the continuous clock is to
     disable the peripheral by SPE=0. This must occur in specific time window within last data
     frame transaction just between the sampling time of its first bit and before its last bit transfer
     starts (in order to receive a complete number of expected data frames and to prevent any
     additional “dummy” data reading after the last valid data frame). Specific procedure must be
     followed when disabling SPI in this mode. 
  */
  if(!spi_line)
    return SPI_ERR_INVALID_PARAM;

  // Get Previous SPI_IRQ status, disable interrupts for the critical section
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  uint32_t bsy_timeout = SPI_TXE_TIMEOUT;

  //volatile uint32_t rxfifo;
  if(rxonly != SPI_RECEIVE_ONLY){
    uint32_t ftlvl_timeout = SPI_TXE_TIMEOUT;
    // Wait until FTLVL = 0
    if(wait_until_clear(&spi_line->SR, SPI_SR_FTLVL, ftlvl_timeout) == SPI_ERR_TIMEOUT){ // If timeout
      // Restore SPI IRQ state
      if(!primask)
        __enable_irq();
      
      return SPI_ERR_TIMEOUT;
    }
    // Wait until BSY = 0
    if(wait_until_clear(&spi_line->SR, SPI_SR_BSY, bsy_timeout) == SPI_ERR_TIMEOUT) { // If timeout
      // Restore SPI IRQ state
      if(!primask)
        __enable_irq();
      
      return SPI_ERR_TIMEOUT;
    }

    // Disable SPI
    spi_line->CR1 &= ~SPI_CR1_SPE;
    __DSB();
    __ISB();

    // Read data until FRLVL[1:0] = 00 (read all the reveived data)
    while(spi_line->SR & SPI_SR_FRLVL){
      volatile uint8_t dummy = *(volatile uint8_t *)&spi_line->DR;
      (void)dummy;
    } 
  }
  else{ // rxonly mode 
    // TODO: Interrupt the receive flow by disabling SPI in the specific time window while the last data frame is ongoing

    // Wait until BSY = 0
    if(wait_until_clear(&spi_line->SR, SPI_SR_BSY, bsy_timeout) == SPI_ERR_TIMEOUT){
      // Restore SPI IRQ state
      if(!primask)
        __enable_irq();

      return SPI_ERR_TIMEOUT;
    }

    // Disable SPI
    spi_line->CR1 &= ~SPI_CR1_SPE;
    __DSB();
    __ISB();

    // Read data until FRLVL[1:0] = 00 (read all the reveived data)
    while(spi_line->SR & SPI_SR_FRLVL) {
      volatile uint8_t dummy = *(volatile uint8_t *)&spi_line->DR;
      (void)dummy;
    }
  }

  // If IRQs were previously enabled, enable
  if(!primask)
    __enable_irq();

  return SPI_OK;
}

/**
 * @brief A Non-blocking SPI-Disable Function that sets flags and uses interrupts to complete the proper disable procedure
 * @param spi_line The SPI peripheral to disable 
 * @param rxonly Indicates whether the SPI peripheral is configured in rxonly mode (including the read only setting in half-duplex mode)
 * 
 * @retval An error code of form SPI_ERR_X, or SPI_OK if no error occured
 */
int8_t spi_disable_nonblocking(SPI_TypeDef* spi_line, uint8_t rxonly){
  if(!spi_line) return SPI_ERR_INVALID_PARAM;

  spi_context_t* ctx = spi_get_context(spi_line);
  if(!ctx) return SPI_ERR_INVALID_PARAM;

  // Get Previous SPI_IRQ status, disable interrupts for the critical section
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  ctx->rxonly = rxonly;
  
  ctx->state = SPI_STATE_SHUTDOWN_PENDING;

  if(!ctx->using_dma){ // If DMA is being used, DMA TC will come first -> Manager calls spi_on_dma_done()
    spi_line->CR2 |= SPI_CR2_TXEIE; // Ensure TXE interrupt is enabled
    if((spi_line->SR & SPI_SR_FTLVL_Msk) == 0){ // If TXFIFO is already empty proceed to next step in shutdown
      ctx->state = SPI_STATE_SHUTDOWN_TX_EMPTIED;
      spi_check_busy_and_disable(spi_line, ctx);
    }
  }
  // Else: DMA will signal TC in it's IRQ -> Manager calls spi_on_dma_done()

  spi_line->CR2 |= SPI_CR2_RXNEIE; // Ensure RXNE interrupt is enabled

  // If IRQs were previously enabled, enable
  if(!primask)
    __enable_irq();

  return SPI_OK;
}

int8_t spi_disable_dmatxen(SPI_TypeDef* spi_line){
  if(!spi_line) return SPI_ERR_INVALID_PARAM;

  spi_line->CR2 &= ~SPI_CR2_TXDMAEN;

  return SPI_OK;
}

int8_t spi_disable_dmarxen(SPI_TypeDef* spi_line){
  if(!spi_line) return SPI_ERR_INVALID_PARAM;

  spi_line->CR2 &= ~SPI_CR2_RXDMAEN;
  
  return SPI_OK;
}

// int8_t spi_on_dma_done(SPI_TypeDef* spi_line){
//   // If DMA has signalled TC - 
// }

/**
 * @brief Small helper function that checks whether spi is enabled
 * @param spi_line The spi peripheral to check
 * 
 * @retval 1 - spi IS enabled  0 - spi IS NOT enabled, <0: An error occured
 */
int8_t spi_is_enabled(const SPI_TypeDef* spi_line){
  if(!spi_line)
    return SPI_ERR_INVALID_PARAM;

  if (spi_line->CR1 & SPI_CR1_SPE)
    return 1;
  else
    return 0;
}

/**
 * @brief Assert the NSS signal for the intended peripheral
 * @param spi_line The SPI peripheral that will drive the NSS signal
 * 
 * @note NSS signal asserts low
 * @retval An error code of form SPI_ERR_X, or SPI_OK if no error occured
 */
int8_t spi_assert_nss(SPI_TypeDef* spi_line){
  if(!spi_line)
    return SPI_ERR_INVALID_PARAM;
  
  // Get Previous IRQ status, disable interrupts for the critical section
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  if (!(spi_line->CR1 & SPI_CR1_SSM)) // If software NSS management is disabled - enable it
    spi_line->CR1 |= SPI_CR1_SSM;

  spi_line->CR1 &= ~SPI_CR1_SSI; // NSS asserts low


  // If IRQs were previously enabled, enable
  if(!primask)
    __enable_irq();


  return SPI_OK;
}


/**
 * @brief Did I have to write another function? No.
 */
int8_t spi_deassert_nss(SPI_TypeDef* spi_line){
  if(!spi_line)
    return SPI_ERR_INVALID_PARAM;

  // Get Previous IRQ status, disable interrupts for the critical section
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  if (!(spi_line->CR1 & SPI_CR1_SSM)) // If software NSS management is disabled - enable it
    spi_line->CR1 |= SPI_CR1_SSM;

  spi_line->CR1 |= SPI_CR1_SSI;

  // If IRQs were previously enabled, enable
  if(!primask)
    __enable_irq();


  return SPI_OK;
}

/*=================== SPI TRANSMITTER FUNCTIONS ===================*/

/**
 * @brief A Blocking Function for Transmitting Data on the SPI peripheral
 * @param spi_line The SPI peripheral to transmit on
 * @param data_packet Pointer to a struct containing data + metadata (data size, buffer size)
 * @retval An error code of form SPI_ERR_X, or SPI_OK if there is no error
 * 
 * @note This function is blocking, If you want to read as you write this function cannot be used
 * @note You must make sure that data_packet->data_size is the same as the data size configured in CR2
 */
int8_t spi_transmit_polling(SPI_TypeDef* spi_line, const spi_data_packet_t* data_packet){
  // Validation
  if(!spi_line || !data_packet || !data_packet->buffer)
    return SPI_ERR_INVALID_PARAM;

  if(data_packet->data_size > SPI_DATA_SIZE_MAX ||
     data_packet->data_size < SPI_DATA_SIZE_MIN)
     return SPI_ERR_INVALID_PARAM;

  // Ensure SPI is enabled
  if (!(spi_line->CR1 & SPI_CR1_SPE)) {
    spi_line->CR1 |= SPI_CR1_SPE;
    __DSB();
  }

  // Transmit Data
  if(data_packet->data_size <= SPI_DATA_SIZE_CHAR){ // If the data size is <= 8-bits, represent as uint8_t
    const uint8_t* data = (const uint8_t*)data_packet->buffer;
    for(uint32_t i = 0; i < data_packet->buffer_size; i++){
      uint32_t timeout = SPI_TXE_TIMEOUT;
      while(!(spi_line->SR & SPI_SR_TXE)){      //Wait for space in the TXFIFO buffer
        if(--timeout == 0) {
          return SPI_ERR_TIMEOUT;
        }
      }
      *(volatile uint8_t *)&spi_line->DR = data[i];
      
      // Clear received data
      timeout = SPI_RXNE_TIMEOUT;
      while(!(spi_line->SR & SPI_SR_RXNE)){
        if(--timeout == 0)
          return SPI_ERR_TIMEOUT;
      }

      volatile uint8_t dummy = *(volatile uint8_t *)&spi_line->DR;
      (void)dummy;
    }
  }
  else{                                             // If the data size is > 8-bits, represent as uint16_t
    const uint16_t* data = (const uint16_t*)data_packet->buffer;
    for(uint32_t i = 0; i < data_packet->buffer_size; i++){
      uint32_t timeout = SPI_TXE_TIMEOUT;
      while(!(spi_line->SR & SPI_SR_TXE)){      //Wait for space in the TXFIFO buffer
        if(--timeout == 0)
          return SPI_ERR_TIMEOUT;
      }
      spi_line->DR = data[i];
      
      // Clear received data
      timeout = SPI_RXNE_TIMEOUT;
      while(!(spi_line->SR & SPI_SR_RXNE)){
        if(--timeout == 0)
          return SPI_ERR_TIMEOUT;
      }

      volatile uint16_t dummy = spi_line->DR;
      (void)dummy;
    }
  }

  uint32_t timeout = SPI_TXE_TIMEOUT;
  while(spi_line->SR & SPI_SR_BSY){
    if(--timeout == 0){
      return SPI_ERR_TIMEOUT;
    }
  }

  return SPI_OK;
}

/**
 * @brief Begin the SPI interrupt based transmission process
 * @param spi_line The SPI peripheral to start transmitting on
 * @param data_packet Pointer to a struct containing the buffer, buffer-size and data-size
 * 
 * @note This function does not setup the receiver functionality, this should be done before calling this function
 * @retval An error code of form SPI_ERR_X, or SPI_OK if there is no error
 */
int8_t spi_transfer_interrupts(SPI_TypeDef* spi_line, const spi_data_packet_t* data_packet){
  if(!spi_line || !data_packet || !data_packet->buffer)
    return SPI_ERR_INVALID_PARAM;

  if(!data_packet->buffer_size ||
     data_packet->data_size < SPI_DATA_SIZE_MIN ||
     data_packet->data_size > SPI_DATA_SIZE_MAX)
    return SPI_ERR_INVALID_PARAM;

  // Update SPI context
  spi_context_t* ctx = spi_get_context(spi_line);
  if(!ctx) return SPI_ERR_INVALID_PARAM;

  ctx->tx_buffer = data_packet->buffer;
  ctx->tx_buffer_size = data_packet->buffer_size;
  ctx->tx_index = 0;
  ctx->tx_in_progress = 1;

  write_bits(&spi_line->CR2, SPI_CR2_TXEIE_Msk, SPI_CR2_TXEIE_Pos, SPI_CR2_TXEIE);

  return SPI_OK;
}

/**
 * @brief Helper function to be used in conjunction with spi_transmit_dma & DMA functions in order to communicate via DMA
 * @param spi_line SPI peripheral to configure
 * @param en the bit to be set. Any value > 1 will be treated as 1. If 0 is passed, dmatxen will be cleared
 * @retval an error code of form SPI_ERR_X, or SPI_OK if there is no error
 */
int8_t spi_set_dmatxen(SPI_TypeDef* spi_line, uint8_t en){
  if(!spi_line)
    return SPI_ERR_INVALID_PARAM;
  
  write_bits(&spi_line->CR2, SPI_CR2_TXDMAEN_Msk, SPI_CR2_TXDMAEN_Pos, en ? 1U : 0U);

  return SPI_OK;
}


/**
 * @brief This is a blocking function that transmits and receives
 */
int8_t spi_transfer_polling(SPI_TypeDef* spi_line,
                            const void* tx_buffer,
                            void* rx_buffer,
                            uint32_t length,
                            uint8_t data_size) {
    if (!spi_line || !tx_buffer || !rx_buffer) return SPI_ERR_INVALID_PARAM;

    if(data_size < SPI_DATA_SIZE_MIN ||
       data_size > SPI_DATA_SIZE_MAX)
       return SPI_ERR_INVALID_PARAM;

    
       // Misalignment check
    if (data_size > SPI_DATA_SIZE_CHAR) {
      if (((uintptr_t)tx_buffer & 1) || ((uintptr_t)rx_buffer & 1)) 
        return SPI_ERR_INVALID_PARAM;
    }

    // Ensure SPI is enabled
    if (!(spi_line->CR1 & SPI_CR1_SPE)) {
      spi_line->CR1 |= SPI_CR1_SPE;
      __DSB();
    }

    for (uint32_t i = 0; i < length; i++) {
        // Wait until TX buffer empty
      uint32_t timeout = SPI_TXE_TIMEOUT;
      while (!(spi_line->SR & SPI_SR_TXE)) {
          if (--timeout == 0) return SPI_ERR_TIMEOUT;
      }

      // Write next byte
      if(data_size <= SPI_DATA_SIZE_CHAR){
        const uint8_t* char_buffer = (uint8_t*)tx_buffer;
        *(volatile uint8_t *)&spi_line->DR = char_buffer[i];
      }
      else{
        const uint16_t* short_buffer = (uint16_t*)tx_buffer;
        spi_line->DR = short_buffer[i];
      }


      // Wait until RX buffer has data
      timeout = SPI_RXNE_TIMEOUT;
      while (!(spi_line->SR & SPI_SR_RXNE)) {
          if (--timeout == 0) return SPI_ERR_TIMEOUT;
      }      

      // Read received byte
      if(data_size <= SPI_DATA_SIZE_CHAR){
        uint8_t rx = *(volatile uint8_t *)&spi_line->DR;
        if (i > 0){
          ((uint8_t *)rx_buffer)[i - 1] = rx;
        }
      }
      else{
        uint16_t rx = spi_line->DR;
        if (i > 0){
          ((uint16_t *)rx_buffer)[i - 1] = rx;
        }
      }
    }

    // Ensure last bit is shifted
    uint32_t timeout = SPI_TXE_TIMEOUT;
    while (spi_line->SR & SPI_SR_BSY) {
        if (--timeout == 0) return SPI_ERR_TIMEOUT;
    }

    return SPI_OK;
}


/*=================== SPI RECEIVER FUNCTIONS ===================*/

/**
 * @brief Helper function to be used in conjunction with DMA functions in order to communicate via DMA
 * @param spi_line SPI peripheral to configure
 * @param en The bit to be set. Any value > 1 will be treated as 1. If 0 is passed, dmarxen will be cleared
 * 
 * @retval an error code of form SPI_ERR_X, or SPI_OK if there is no error
 */
int8_t spi_set_dmarxen(SPI_TypeDef* spi_line, uint8_t en){
  if(!spi_line)
    return SPI_ERR_INVALID_PARAM;

  write_bits(&spi_line->CR2, SPI_CR2_RXDMAEN_Msk, SPI_CR2_RXDMAEN_Pos, en ? 1U : 0U);

  return SPI_OK;
}


// ===================== SPI ISR Code  ==========================//

/**
 * @brief Called in disable procedure, if BSY = 0, we disable the SPI peripheral and read the remainder of RXFIFO.
 * @param spi_line The SPI peripheral to check is busy
 * @retval 1 - SPI is busy
 * @retval 0 - SPI is not busy
 * @retval <0 - An error code of form SPI_ERR_X
 */
static int8_t spi_check_busy_and_disable(SPI_TypeDef* spi_line, spi_context_t* ctx){
  if(!spi_line) return SPI_ERR_INVALID_PARAM;

  if((spi_line->SR & SPI_SR_BSY ) == 0U){
    // If RXFIFO is already empty - disable SPI
    if(!(spi_line->SR & SPI_SR_FRLVL)){
      spi_line->CR1 &= ~SPI_CR1_SPE;
      __DSB();
      __ISB();
      ctx->state = SPI_STATE_IDLE;
    }
    else{ // Otherwise, enable RXNE interrupts to drain RXFIFO
      ctx->state = SPI_STATE_SHUTDOWN_BSY_0;
      spi_line->CR2 |= SPI_CR2_RXNEIE;
    }
    return 0;
  }
  return 1;
}

/**
 * @brief TXE Event Handler, Called by IRQx_IRQHandler() on a TXE interrupt.
 * @brief This ordinarily just writes data from a given buffer into the data register.
 * @brief In the event this is called during the SPI disable procedure this function will check FTLVL[1:0] == 0.
 * @brief If the transmission register is empty it will proceed to the next state in the shutdown procedure
 * 
 * @param spi_line The SPI peripheral to handle the TXE event on
 * @param ctx Pointer to the SPI peripherals context struct, containing information like states, buffers and indexes
 * @param data_size Size of a data frame in bits
 * 
 * @retval an error code of form SPI_ERR_X, or SPI_OK if there is no error
 */
static int8_t spi_handle_txe(SPI_TypeDef* spi_line, spi_context_t* ctx, uint8_t data_size){
  if(!spi_line || !ctx)return SPI_ERR_INVALID_PARAM;

  if(data_size < SPI_DATA_SIZE_MIN ||
     data_size > SPI_DATA_SIZE_MAX)
    return SPI_ERR_INVALID_PARAM;

  uint32_t sr = spi_line->SR;

  if ((ctx->tx_index < ctx->tx_buffer_size) && ctx->tx_in_progress){ // If there is data to be read
    if(data_size <= SPI_DATA_SIZE_CHAR){ // If data can be represented as a char, cast to uint8_t
      const uint8_t* char_buffer = (const uint8_t*)ctx->tx_buffer;
      // Write to data register & increment the index
      spi_line->DR = char_buffer[ctx->tx_index++];
    }
    else{ // If data cannot be represented as a char, cast to uint16_t -- range validation already done
      const uint16_t* short_buffer = (const uint16_t*)ctx->tx_buffer;
      // Write to data register & increment the index
      spi_line->DR = short_buffer[ctx->tx_index++];
    }
  }

  if (ctx->state == SPI_STATE_SHUTDOWN_PENDING){ // If caller has started disabling spi 
    if ((sr & SPI_SR_FTLVL) == 0U){ // If FTLVL[1:0] == 00 (TXFIFO is empty)
      // Next State in the shutdown procedure
      ctx->state = SPI_STATE_SHUTDOWN_TX_EMPTIED;
      ctx->tx_in_progress = 0; 

      // If BSY = 0, disable SPI and read the remainder of the RXFIFO
      spi_check_busy_and_disable(spi_line, ctx);

      return SPI_OK;
    }
    // Else TXFIFO is not empty, leave TXEIE enabled & return
    return SPI_OK;
  }

  //If transmission is finished disable interrupts
  if(!ctx->tx_in_progress)
    spi_line->CR2 &= ~SPI_CR2_TXEIE;

  return SPI_OK;
}

/**
 * @brief This function reads the data in the RXFIFO register into a given buffer
 * @param spi_line The SPI peripheral to read the data on
 * @param ctx Pointer to a struct with information on the SPI peripherals context (buffer poistions, states)
 * @param data_size The size of the data in bits
 * 
 * @note This function is intended to be called when the rxne flag is set, do not call it from another context
 */
static int8_t spi_handle_rxne(SPI_TypeDef* spi_line, spi_context_t* ctx, uint8_t data_size){
  if(!spi_line || !ctx) return SPI_ERR_INVALID_PARAM;

  if(data_size < SPI_DATA_SIZE_MIN ||
     data_size > SPI_DATA_SIZE_MAX)
    return SPI_ERR_INVALID_PARAM;

  //uint32_t sr = spi_line->SR;

  if(ctx->rx_index < ctx->rx_buffer_size){
    if(data_size <= SPI_DATA_SIZE_CHAR){
      uint8_t* char_buffer = (uint8_t*)ctx->rx_buffer; // If data can be represented as a char, cast to uint8_t
      uint8_t character = spi_line->DR;
      char_buffer[ctx->rx_index++] = character; 
    }
    else{
      uint16_t* short_buffer = (uint16_t*)ctx->rx_buffer; // If data cannot be represented as a char, cast to uint16_t
      uint16_t data  = spi_line->DR;
      short_buffer[ctx->rx_index++] = data; 
    }
  }
  else{
    // Read and discard data to avoid OVR
    if(data_size <= SPI_DATA_SIZE_CHAR){
      uint8_t tmp = spi_line->DR;
      (void)tmp;
    }
    else{
      uint16_t data  = spi_line->DR;
      (void)data;
    }
  }

  // If BSY = 0 (the first two steps of the disable procedure are complete)
  if (ctx->state == SPI_STATE_SHUTDOWN_BSY_0){
    // If RXFIFO is empty, Disable SPI
    if (!(spi_line->SR &  SPI_SR_FRLVL)){
      spi_line->CR1 &= ~SPI_CR1_SPE;
      __DSB();
      __ISB();
    }
  }

  return SPI_OK;
}

/**
 * @brief This is the generic SPI ISR, it handles error events and txe/rxne events 
 * @param spi_line The SPI peripheral to handle the event on
 * 
 * @note This is not a ISR defined in the startup file, it must be called from one of those IRQHandlers e.g. SPI1_IRQHandler{SPIx_IRQHandler(SPI1)}
 */
static int8_t SPIx_IRQHandler(SPI_TypeDef* spi_line, spi_context_t* ctx, uint8_t data_size){
  if (!spi_line || !ctx) return SPI_ERR_INVALID_PARAM;

  uint32_t sr = spi_line->SR;

  if(sr & SPI_SR_OVR){ // If an overrun error has occured
    // Handle Overrun Error
    uint32_t tmp1 = spi_line->DR;
    (void)tmp1;
    uint32_t tmp2 = spi_line->SR;
    (void)tmp2;
    // Possibly Notify error callback
  }
  
  if(sr & SPI_SR_MODF){ // If a Mode Fault has occured - This is a temporary solution
    // This flag is set by hardware and reset by a software sequence. Refer to Section : Mode fault (MODF) on page 1347 for the software sequence.
    // Read from SR
    uint32_t tmp = spi_line->SR;
    (void)tmp;
    //write to CR1 register
    spi_line->CR1 |= SPI_CR1_SPE;
    // Possibly Notify fault callback
  }
  
  if((sr & SPI_SR_RXNE) && (spi_line->CR2 & SPI_CR2_RXNEIE) && ctx->rx_buffer){ // If the receive buffer isn't empty & rxne interrupts are enabled + rx_buffer null ptr check
    spi_handle_rxne(spi_line, ctx, data_size);
  }
  
  // Handle TXE Flag 
  if((sr & SPI_SR_TXE) && (spi_line->CR2 & SPI_CR2_TXEIE) && ctx->tx_buffer){
    spi_handle_txe(spi_line, ctx, data_size);
  }

  return SPI_OK;
}

void SPI1_IRQHandler(){
  spi_context_t* spi1_ctx = spi_get_context(SPI1);
  uint8_t data_size = ((SPI1->CR2 & SPI_CR2_DS) >> SPI_CR2_DS_Pos) + 1; // DS stores a data-size of 4-bits as 0011, 5-bits as 0100 etc...
  SPIx_IRQHandler(SPI1, spi1_ctx, data_size);
}

void SPI3_IRQHandler(){
  spi_context_t* spi3_ctx = spi_get_context(SPI3);
  uint8_t data_size = ((SPI3->CR2 & SPI_CR2_DS) >> SPI_CR2_DS_Pos) + 1; // DS stores a data-size of 4-bits as 0011, 5-bits as 0100 etc...
  SPIx_IRQHandler(SPI3, spi3_ctx, data_size);
}