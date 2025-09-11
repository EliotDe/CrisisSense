/**
 * @brief This Is the SPI Driver
 * @todo Add __DSBs, __ISBs where needed
 */

#include "stm32l432xx.h"
#include "spi_driver.h"
#include <stddef.h>

#define SPI_CR1_LENGTH 32u
#define SPI_CR2_LENGTH 16u

#define SPI_DATA_SIZE_MIN 4u
#define SPI_DATA_SIZE_CHAR 8u
#define SPI_DATA_SIZE_MAX 16u

#define SPI_RXNE_TIMEOUT 10000u
#define SPI_TXE_TIMEOUT 50000u

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
  SPI_STATE_SHUTDOWN_PENDING,
  SPI_STATE_SHUTDOWN_TX_EMPTIED
}spi_state_t;

typedef struct{
  SPI_TypeDef* spi;
  volatile spi_state_t state;
  uint8_t rxonly;
  uint8_t using_dma;
  uint8_t tx_in_progress;
  uint32_t deadline_ms; // Timeout (non-blocking) -- optional
  const void* tx_buffer;
  void* rx_buffer;
  uint32_t tx_buffer_size;
  uint32_t rx_buffer_size;
  uint32_t tx_index;
  uint32_t rx_index;
  uint32_t expected_rx_frames;
  //uint32_t rx_count;
}spi_context_t;


static volatile spi_context_t spi_states[2] = {// For SPI1 and SPI3 -- Should it be Volatile?
  {.spi = SPI1, .state = SPI_STATE_IDLE}, //SPI1 - initial context
  {.spi = SPI3, .state = SPI_STATE_IDLE}  //SPI3 - initial context
}; 

static inline spi_context_t* spi_get_context(SPI_TypeDef* spi_line);
static int8_t spi_handle_txe(SPI_TypeDef* spi_line, spi_context_t* ctx, uint8_t data_size);
static int8_t spi_handle_rxne(SPI_TypeDef* spi_line, spi_context_t* ctx, uint8_t data_size);
static void SPIx_IRQHandler(SPI_TypeDef* spi_line, spi_context_t* ctx, uint8_t rxonly, uint8_t data_size);

/*============================ SPI HELPER FUNCTIONS ======================================*/

/**
 * @brief This helper function inputs the spi peripheral and returns the pointer to its context struct
 * @param spi_line The SPI peripheral 
 * @retval The pointer to the SPI peripherals context struct
 * @retval NULL - if an Invalid spi_line is passed, e.g. SPI1276 
 */
static inline spi_context_t* spi_get_context(SPI_TypeDef* spi_line){
  if(spi_line == SPI1) return &spi_states[0];
  if(spi_line == SPI3) return &spi_states[1];
  return NULL; // Invalid spi_line passed
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
  uint32_t cr1 = spi_line->CR1;
  
  // CURRENTLY DOESN'T HANDLE ERRORS RECEIVED FROM WRITE_BITS
  write_bits(&cr1, SPI_CR1_LENGTH, (uint32_t)SPI_CR1_BR_Msk, (uint32_t)SPI_CR1_BR_Pos, cfg->baud_rate_factor);
  write_bits(&cr1, SPI_CR1_LENGTH, (uint32_t)SPI_CR1_CPOL_Msk, (uint32_t)SPI_CR1_CPOL_Pos, cfg->clock_polarity);
  write_bits(&cr1, SPI_CR1_LENGTH, (uint32_t)SPI_CR1_CPHA_Msk, (uint32_t)SPI_CR1_CPHA_Pos, cfg->clock_phase);
  write_bits(&cr1, SPI_CR1_LENGTH, (uint32_t)SPI_CR1_BIDIMODE_Msk, (uint32_t)SPI_CR1_BIDIMODE_Pos, cfg->bidirectional_mode); 
  write_bits(&cr1, SPI_CR1_LENGTH, (uint32_t)SPI_CR1_BIDIOE_Msk, (uint32_t)SPI_CR1_BIDIOE_Pos, cfg->bidi_direction);
  write_bits(&cr1, SPI_CR1_LENGTH, (uint32_t)SPI_CR1_RXONLY_Msk, (uint32_t)SPI_CR1_RXONLY_Pos, cfg->simplex_mode);
  write_bits(&cr1, SPI_CR1_LENGTH, (uint32_t)SPI_CR1_LSBFIRST_Msk, (uint32_t)SPI_CR1_LSBFIRST_Pos, cfg->lsb_first);
  write_bits(&cr1, SPI_CR1_LENGTH, (uint32_t)SPI_CR1_CRCL_Msk, (uint32_t)SPI_CR1_CRCL_Pos, cfg->crc_code_length);
  write_bits(&cr1, SPI_CR1_LENGTH, (uint32_t)SPI_CR1_CRCEN_Msk, (uint32_t)SPI_CR1_CRCEN_Pos, cfg->enable_crc);
  write_bits(&cr1, SPI_CR1_LENGTH, (uint32_t)SPI_CR1_SSM_Msk, (uint32_t)SPI_CR1_SSM_Pos, cfg->ssm_enabled);
  write_bits(&cr1, SPI_CR1_LENGTH, (uint32_t)SPI_CR1_SSI_Msk, (uint32_t)SPI_CR1_SSI_Pos, cfg->ssi_bit);
  write_bits(&cr1, SPI_CR1_LENGTH, (uint32_t)SPI_CR1_MSTR_Msk, (uint32_t)SPI_CR1_MSTR_Pos, cfg->master_selction);
  
  spi_line->CR1 = cr1;

  // Configure SPI_CR2 register
  uint16_t cr2 = spi_line->CR2;

  uint8_t data_size = cfg->data_size_in_bits;
  uint8_t data_size_bits = data_size - 1;       // DS[3:0] is stored like this (0011: 4-bit, 0100: 5-bit, ..., 1111: 16-bit)
  write_bits(&cr2, SPI_CR2_LENGTH, (uint32_t)SPI_CR2_DS_Msk, (uint32_t)SPI_CR2_DS_Pos, data_size_bits);
  write_bits(&cr2, SPI_CR2_LENGTH, (uint32_t)SPI_CR2_SSOE_Msk, (uint32_t)SPI_CR2_SSOE_Pos, cfg->slave_select_enabled);
  write_bits(&cr2, SPI_CR2_LENGTH, (uint32_t)SPI_CR2_FRF_Msk, (uint32_t)SPI_CR2_FRF_Pos, cfg->frame_format);
  write_bits(&cr2, SPI_CR2_LENGTH, (uint32_t)SPI_CR2_NSSP_Msk, (uint32_t)SPI_CR2_NSSP_Pos, cfg->nss_pulse);
  write_bits(&cr2, SPI_CR2_LENGTH, (uint32_t)SPI_CR2_FRXTH_Msk, (uint32_t)SPI_CR2_FRXTH_Pos, cfg->fifo_rx_threshold);
  write_bits(&cr2, SPI_CR2_LENGTH, (uint32_t)SPI_CR2_LDMATX_Msk, (uint32_t)SPI_CR2_LDMATX_Pos, cfg->dma_tx_data_length);
  write_bits(&cr2, SPI_CR2_LENGTH, (uint32_t)SPI_CR2_LDMARX_Msk, (uint32_t)SPI_CR2_LDMARX_Pos, cfg->dma_rx_data_length);

  spi_line->CR2 = cr2;
  
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
  uint32_t cr1 = spi_line->CR1;
  cr1 |= SPI_CR1_SPE;
  spi_line->CR1 = cr1;


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

  uint32_t rxfifo;
  if(rxonly != SPI_RECEIVE_ONLY){
    while(spi_line->SR & SPI_SR_FTLVL); // Wait until FTLVL[1:0] = 00
    while(spi_line->SR & SPI_SR_BSY);   // Wait until BSY=0
    spi_line->CR1 &= ~(SPI_CR1_SPE_Msk << SPI_CR1_SPE_Pos); //Disable SPI
    while(spi_line->SR & SPI_SR_FRLVL) // Read data until FRLVL[1:0] = 00 (read all the reveived data)
      rxfifo = spi_line->DR;
  }
  else if (rxonly == SPI_RECEIVE_ONLY){
    // TODO: Interrupt the receive flow by disabling SPI in the specific time window while the last data frame is ongoing
    while(spi_line->SR & SPI_SR_BSY);
    while(spi_line->SR & SPI_SR_FRLVL) // Read data until FRLVL[1:0] = 00 (read all the reveived data)
      rxfifo = spi_line->DR;
  }

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

  ctx->rxonly = rxonly;
  ctx->deadline_ms = SPI_RXNE_TIMEOUT; //placeholder
  // ctx->rx
  
  ctx->state = SPI_STATE_SHUTDOWN_PENDING;

  if(!ctx->using_dma) // If DMA is being used, DMA TC will come first -> Manager calls spi_on_dma_done()
    spi_line->CR2 |= SPI_CR2_TXEIE; // Ensure TXE interrupt is enabled
  else 
    // DMA will signal TC in it's IRQ -> Manager calls spi_on_dma_done()

  spi_line->CR2 |= SPI_CR2_RXNEIE; // Ensure RXNE interrupt is enabled

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
int8_t spi_is_enabled(SPI_TypeDef* spi_line){
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
  
  // Critical Section
  // Get SPI1_IRQn & SPI3_IRQn interrupt status (enabled/disabled)
  // Disable Interrupts - Consider making an __NVIC_Disable_SPI_IRQ() or __NVIC_Disable_IRQs()
  __NVIC_DisableIRQ(SPI1_IRQn);
  __NVIC_DisableIRQ(SPI3_IRQn);

  if (!(spi_line->CR1 & SPI_CR1_SSM)) // If software NSS management is disabled - enable it
    spi_line->CR1 |= SPI_CR1_SSM;

  write_bits(&spi_line->CR1, SPI_CR1_LENGTH, SPI_CR1_SSI_Msk, SPI_CR1_SSI_Pos, ~SPI_CR1_SSI); // NSS asserts low

  // Enable Interrupts - "" ""
  // Consider if this function is called from within another critical section - This would enable interrupts in the middle of it
  // Although i doubt it will best to be careful
  // if SPI1_IRQn & SPI3_IRQn original status was disabled, leave it disabled. Otherwise:
  __NVIC_EnableIRQ(SPI1_IRQn);
  __NVIC_EnableIRQ(SPI3_IRQn);

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

  if(spi_line->CR1 & SPI_CR1_SPE)
    return SPI_ERR_BUSY;

  spi_line->CR1 |= SPI_CR1_SPE;

  // Transmit Data
  uint32_t timeout = SPI_TXE_TIMEOUT;
  if(data_packet->data_size <= SPI_DATA_SIZE_CHAR){ // If the data size is <= 8-bits, represent as uint8_t
    const uint8_t* data = (const uint8_t*)data_packet->buffer;
    for(uint8_t i = 0; i < data_packet->buffer_size; i++){
      while(!(spi_line->SR & SPI_SR_TXE)){      //Wait for space in the TXFIFO buffer
        if(--timeout == 0) {
          return SPI_ERR_TIMEOUT;
        }
      }
      spi_line->DR = data[i];
    }
  }
  else{                                             // If the data size is > 8-bits, represent as uint16_t
    const uint16_t* data = (const uint16_t*)data_packet->buffer;
    for(uint8_t i = 0; i < data_packet->buffer_size; i++){
      while(!(spi_line->SR & SPI_SR_TXE)){      //Wait for space in the TXFIFO buffer
        while(!(spi_line->SR & SPI_SR_RXNE)){
        if(--timeout == 0)
          return SPI_ERR_TIMEOUT;
      }
      }
      spi_line->DR = data[i];
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
  ctx->tx_buffer = data_packet->buffer;
  ctx->tx_buffer_size = data_packet->buffer_size;
  //data_size = data_packet->data_size;
  ctx->tx_index = 0;
  ctx->tx_in_progress = 1;

  write_bits(&spi_line->CR1, SPI_CR1_LENGTH, SPI_CR2_TXEIE_Msk, SPI_CR2_TXEIE_Pos, SPI_CR2_TXEIE);

  return SPI_OK;
}

/**
 * @brief Helper function to be used in conjunction with spi_transmit_dma & DMA functions in order to communicate via DMA
 * @param spi_line SPI peripheral to configure
 * @param en he bit to be set. Any value > 1 will be treated as 1. If 0 is passed, dmatxen will be cleared
 * @retval an error code of form SPI_ERR_X, or SPI_OK if there is no error
 */
int8_t spi_set_dmatxen(SPI_TypeDef* spi_line, uint8_t en){
  if(!spi_line)
    return SPI_ERR_INVALID_PARAM;
  
  write_bits(&spi_line->CR2, SPI_CR2_LENGTH, SPI_CR2_TXDMAEN_Msk, SPI_CR2_TXDMAEN_Pos, en ? 1U : 0U);

  return SPI_OK;
}

/*=================== SPI RECEIVER FUNCTIONS ===================*/

/**
 * @brief A Blocking Function for Receiveing Data on the SPI peripheral
 * @param spi_line The SPI peripheral to read from
 * @param buffer The Location in memory to store the received data
 */
int8_t spi_receive_polling(SPI_TypeDef* spi_line, void* buffer, uint8_t data_size){
  // Null ptr Validation
  if(!spi_line || !buffer)
    return SPI_ERR_INVALID_PARAM;

  // Range Validation
  if(data_size < SPI_DATA_SIZE_MIN ||
     data_size > SPI_DATA_SIZE_MAX)
    return SPI_ERR_INVALID_PARAM;

  if(spi_line->CR1 & SPI_CR1_SPE)
    return SPI_ERR_BUSY;
  
  spi_line->CR1 |= SPI_CR1_SPE;

  uint32_t timeout = SPI_RXNE_TIMEOUT;
  uint32_t index = 0;
  if(data_size <= SPI_DATA_SIZE_CHAR){ // If data can be represented as a char, cast to uint8_t
    uint8_t* char_buffer = (uint8_t*)buffer;
    while(spi_line->SR & SPI_SR_BSY){ // While the line is busy and there is data to read
      while(!(spi_line->SR & SPI_SR_RXNE)){
        if(--timeout == 0)
          return SPI_ERR_TIMEOUT;
      }
      char_buffer[index++] = spi_line->DR;
    }
  }
  else if(data_size >  SPI_DATA_SIZE_CHAR){ // If data cannot be represented as a char, cast to uint16_t
    uint16_t* short_buffer = (uint16_t*)buffer;
    while(spi_line->SR & SPI_SR_BSY){ // While the line is busy and there is data to read 
      while(!(spi_line->SR & SPI_SR_RXNE)){
        if(--timeout == 0)
          return SPI_ERR_TIMEOUT;
      }
      short_buffer[index++] = spi_line->DR;
    }
  }

  return SPI_OK;
}


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

  write_bits(&spi_line->CR2, SPI_CR2_LENGTH, SPI_CR2_RXDMAEN_Msk, SPI_CR2_RXDMAEN_Pos, en ? 1U : 0U);

  return SPI_OK;
}


// ===================== SPI ISR Code  ==========================//

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

  uint32_t sr = spi_line->SR;

  if (ctx->tx_index < ctx->tx_buffer_size){ // If there is data to be read
    if(data_size <= SPI_DATA_SIZE_CHAR){ // If data can be represented as a char, cast to uint8_t
      const uint8_t* char_buffer = (const uint8_t*)ctx->tx_buffer;
      // Write to data register & increment the index
      spi_line->DR = char_buffer[ctx->tx_index++];
    }
    else if(data_size > SPI_DATA_SIZE_CHAR){ // If data cannot be represented as a char, cast to uint16_t
      const uint16_t* short_buffer = (const uint16_t*)ctx->tx_buffer;
      // Write to data register & increment the index
      spi_line->DR = short_buffer[ctx->tx_index++];
    }
  }

  if (ctx->state == SPI_STATE_SHUTDOWN_PENDING){ // If caller has started disabling spi 
    if ((sr & SPI_SR_FTLVL) == 0U){ // If FTLVL[1:0] == 01 (one last data frame in TXFIFO)
      // Next State in the shutdown procedure
      ctx->state = SPI_STATE_SHUTDOWN_TX_EMPTIED;
      // Check busy & disable? Or wait for another interrupt that signals BSY = 0;
      return SPI_OK;
    }
    // Else TXFIFO is not empty, leave TXEIE enabled & return
    return SPI_OK;
  }

  // If transmission is finished disable interrupts - This will never be reached
  // if(!ctx->tx_in_progress)
  //   spi_line->CR2 &= ~SPI_CR2_TXEIE;

  // return SPI_OK;
}

static int8_t spi_handle_rxne(SPI_TypeDef* spi_line, spi_context_t* ctx, uint8_t data_size){
  if(ctx->state == SPI_STATE_SHUTDOWN_PENDING){
      // Placeholder
    }
    if(ctx->rx_index < ctx->rx_buffer_size){
      if(data_size <= SPI_DATA_SIZE_CHAR){
        uint8_t* char_buffer = (uint8_t*)ctx->rx_buffer; // If data can be represented as a char, cast to uint8_t
        uint8_t character = spi_line->DR;
        char_buffer[ctx->rx_index++] = character; 
      }
      else if(data_size > SPI_DATA_SIZE_CHAR){
        uint16_t* short_buffer = (uint16_t*)ctx->rx_buffer; // If data cannot be represented as a char, cast to uint16_t
        uint16_t data  = spi_line->DR;
        short_buffer[ctx->rx_index++] = data; 
      }
    }
}

/**
 * @brief This is the generic SPI ISR, it handles error events and txe/rxne events 
 * @param spi_line The SPI peripheral to handle the event on
 * 
 * @note This is not a ISR defined in the startup file, it must be called from one of those IRQHandlers e.g. SPI1_IRQHandler{SPIx_IRQHandler(SPI1)}
 */
static void SPIx_IRQHandler(SPI_TypeDef* spi_line, spi_context_t* ctx, uint8_t rxonly, uint8_t data_size){
  uint32_t sr = spi_line->SR;

  if(sr & SPI_SR_OVR){ // If an overrun error has occured
    // Handle Overrun Error
    uint32_t tmp = spi_line->DR;
    tmp = sr;
    (void)tmp;
    // Possibly Notify error callback
  }
  
  if(sr & SPI_SR_MODF){ // If a Mode Fault has occured
    // This flag is set by hardware and reset by a software sequence. Refer to Section : Mode fault (MODF) on page 1347 for the software sequence.
    uint32_t tmp = sr;
    (void)tmp;
    spi_line->CR1 |= SPI_CR1_SPE; //enable spi
    // Possibly Notify fault callback
  }
  
  if((sr & SPI_SR_RXNE) && (spi_line->CR2 & SPI_CR2_RXNEIE) && ctx->rx_buffer){ // If the receive buffer isn't empty & rxne interrupts are enabled + rx_buffer null ptr check
    spi_handle_rxne(spi_line, ctx, data_size);
  }
  
  // Handle TXE Flag 
  if((sr & SPI_SR_TXE) && (spi_line->CR2 & SPI_CR2_TXEIE) && ctx->tx_buffer){
    spi_handle_txe(spi_line, ctx, data_size);
  }
}

void SPI1_IRQHandler(){
  SPIx_IRQhandler(SPI1);
}

void SPI3_IRQHandler(){
  SPIx_IRQhandler(SPI3);
}