#ifndef SPI_DRIVER_H
#define SPI_DRIVER_H

#include "stm32l432xx.h"
#include <stddef.h>
// placeholder


// SPI error codes
typedef enum{
  SPI_OK = 0,
  SPI_ERR_INVALID_PARAM = -1,
  SPI_ERR_BUSY = -2,
  SPI_ERR_TIMEOUT = -3
}spi_err_t;

// SPI Tx/Rx Modes
typedef enum{
  SPI_POLLING,    // primarily used for debugging
  SPI_INTERRUPTS, // an option for production code
  SPI_DMA         // most promision option for production code
}spi_mode_t;

/*==================== SPI_CR1 Config Enums ===================*/



/**
 * Enumerates SPI Baud Rate Configuration
 * Defined by the factor by which the APB clock is divided
 */
typedef enum{
  SPI_BAUD_DIV2,              //BR[2:0] = 000: Fpclk/2
  SPI_BAUD_DIV4,              //BR[2:0] = 001: Fpclk/4
  SPI_BAUD_DIV8,              //BR[2:0] = 010: Fpclk/8
  SPI_BAUD_DIV16,             //BR[2:0] = 011: Fpclk/16
  SPI_BAUD_DIV32,             //BR[2:0] = 100: Fpclk/32
  SPI_BAUD_DIV64,             //BR[2:0] = 101: Fpclk/64
  SPI_BAUD_DIV128,            //BR[2:0] = 110: Fpclk/128
  SPI_BAUD_DIV256             //BR[2:0] = 111: Fpclk/256
}spi_baud_rate_t;

/**
 * Enumerates SPI Clock Polarity Configuration
 * Defines whether the clock signal is high or low for an IDLE line
 */
typedef enum{
  SPI_CLK_IDLE0,              //CPOL = 0: CK is low when IDLE
  SPI_CLK_IDLE1               //CPOL = 1: CK is high when IDLE
}spi_clock_polarity_t;

/**
 * Enumerates the SPI Clock Phase
 * Defines when the first data capture occurs
 */
typedef enum{
  SPI_CLK_PHASE0,             //CPHA = 0: First data capture edge is the first clock transition
  SPI_CLK_PHASE1              //CPHA = 1: First data capture edge is the second clock transition
}spi_clock_phase_t;


/**
 * Enumerates the SPI communication mode (half-duplex)
 * Enables Half-Duplex communication using a single data line
 * If configuring with SPI_BIDIRECTIONAL, the RXONLY bit must be clear
 */
typedef enum{
  SPI_UNIDIRECTIONAL,         //BIDIMODE = 0: 2-line Unidirectional data mode
  SPI_BIDIRECTIONAL           //BIDIMODE = 1: 1-line bidirectional data mode
}spi_bidimode_t;

/**
 * Enumerates the SPI directionality in half-duplex mode
 * Set this if SPI is configured for half-duplex communication (SPI_BIDIRECTIONAL)
 */
typedef enum{
  SPI_BIDI_RECEIVE_ONLY,      //BIDIOE = 0: Bidirectional line is used for receiving
  SPI_BIDI_TRANSMIT_ONLY      //BIDIOE = 1: Bidirectional line is used for transmitting
}spi_bidi_direction_t;

/**
 * Enumerates the SPI communication mode (simplex) 
 * If Enabling Simplex Mode (SPI_RECEIVE_ONLY) - BIDIMODE must not be set (SPI_BIDIRECTIONAL)
 */
typedef enum{
  SPI_FULL_DUPLEX,            //RXONLY = 0: Simplex mode not enabled, standard Full-Duplex
  SPI_RECEIVE_ONLY            //RXONLY = 1: Simplex mode enabled
}spi_rxonly_t;

/**
 * Enumerates Endianness of SPI Communication
 */
typedef enum{
  SPI_MSB_FIRST,              //LSBFIRST = 0: Data is transmitted/received with MSB first
  SPI_LSB_FIRST               //LSBFIRST = 1: Data is transmitted/received with LSB first
}spi_lsb_first_t;

/**
 * Enumerates the length of the Cyclic Redundancy Check code
 */
typedef enum{
  SPI_CRC_LENGTH_8,           //CRCL = 0: Cyclic Redundancy Check Code is 8 bits
  SPI_CRC_LENGTH_16           //CRCL = 1: Cyclic Redundancy Check Code is 16 bits
}spi_crc_length_t;

/**
 * Enumerates the CRCEN Setting - Cyclic Redundancy Check Enable
 * If enabling (SPI_CRC_ENABLED) - Hardware Calculates a CRC code from data
 */
typedef enum{
  SPI_CRC_DISABLED,           //CRCEN = 0: Cyclic Redundancy Check not enabled
  SPI_CRC_ENABLED             //CRCEN = 1: Cyclic Redundancy Check calculation by hardware
}spi_crc_enable_t;

/**
 * Enumerates Software Slave Mangement enabling bit
 * If Software Slave Management is enabled (SPI_SSM_ENABLED), the NSS pin input is replaced with the SSI bit value
 */
typedef enum{
  SPI_SSM_DISABLED,           //SSM = 0: Software Slave Management is disabled
  SPI_SSM_ENABLED             //SSM = 0: Software Slave Management is enabled
}spi_ssm_enable_t;

/**
 * Enumerates Internal Slave Select bit
 * If Software Slave Mangement is enabled (SPI_SSM_ENABLED) this bit value drives the NSS pin
 */
typedef enum{
  SPI_SSI_LOW = 0,            //SSI = 0: Signal NSS low
  SPI_SSI_HIGH = 1            //SSI = 1: Signal NSS high
}spi_ssi_bit_t;

/**
 * Enumerates Master/Slave Selection
 */
typedef enum{
  SPI_SLAVE_CONFIG,           //MSTR = 0: Slave Configuration
  SPI_MASTER_CONFIG           //MSTR = 1: Master Configuration
}spi_master_selection_t;



/*==================== SPI_CR2 Config Enums ===================*/



// typedef enum{
//   // datasize ?
// }


/**
 * Enumerates the Slave Select output selection (enabled/disabled)
 * If enabled (SPI_SS_OUTPUT_ENABLED) 
 * 
 * @todo Look into this more Please!
 */
typedef enum{
  SPI_SS_OUTPUT_DISABLED,     //SSOE = 0: SS output disabled in master mode; SPI interface can work in multimaster configuration
  SPI_SS_OUTPUT_ENABLED       //SSOE = 1: SS output enabled in master mode; SPI interface cannot work in a multimaster environment
}spi_ss_output_t;

/**
 * Enumerates the Frame Format Mode (SPI Motorola vs SPI TI modes)
 */
typedef enum{
  SPI_MOTOROLA_MODE,          //FRF = 0: Set Frame Format Mode to Motorola
  SPI_TI_MODE                 //FRF = 1: Set Frame Format Mode to TI
}spi_frame_format_t;

/**
 * Enumerates NSS Pulse Configurations
 * Only need to configure in master mode
 * Generates an NSS Pulse between individual data transfers in continuous transfers
 * Forces NSS pin high after the transfer
 * If generating NSS Pulses (SPI_GENERATE_NSS_PULSE) CHPA and TI bits must be kept clear
 */
typedef enum{
  SPI_NO_NSS_PULSE,           //NSSP = 0: No NSS Pulses
  SPI_GENERATE_NSS_PULSE      //NSSP = 1: SPI generates NSS pulses
}spi_nss_pulse_t;

/**
 * @todo look into this more as well!
 * RXFIFO threshold must be aligned to read access size for the SPIx_DR register
 */
typedef enum{
  SPI_FIFO_LVL_HALF,          //FRXTH = 0: RXNE event generates if the FIFO level is >= 1/2 (16-bit)
  SPI_FIFO_LVL_QUARTER        //FRXTH = 1: RXNE event generated if the FIFO level is >= 1/4 (8-bit)   
}spi_fifo_rx_threshold_t;


/**
 * Enumerates the parity of the  DMA_RX Data length (even or odd)
 * Used in data packing mode
 */
typedef enum{
  SPI_DMA_RX_EVEN_DATA,       //LDMA_RX = 0: Number of Data to transfer is even
  SPI_DMA_RX_ODD_DATA         //LDMA_RX = 1: Number of Data to transfer is odd
}spi_dma_rx_data_length_t;

/**
 * Enumerates the parity of the DMA_TX Data length (even or odd)
 * Used in data packing mode
 */
typedef enum{
  SPI_DMA_TX_EVEN_DATA,       //LDMA_TX = 0: Number of Data to transfer is even
  SPI_DMA_TX_ODD_DATA         //LDMA_TX = 1: Number of Data to transfer is odd         
}spi_dma_tx_data_length_t;

/*==================== SPI STRUCTS ====================*/


typedef struct{
  spi_baud_rate_t baud_rate_factor;             //Defines the factor by which the APB clock is divided
  spi_clock_polarity_t clock_polarity;          //Defines whether the clock signal is high or low for an IDLE line
  spi_clock_phase_t clock_phase;                //Defines when the first data capture occurs
  spi_bidimode_t bidirectional_mode;            //Choose between 2-unidirectional lines, or 1-bidirectional line
  spi_bidi_direction_t bidi_direction;          //When half-duplex mode is enbled (.bidirectional_mode = SPI_BIDIRECTIONAL), this configures the direction on the line
  spi_rxonly_t simplex_mode;                    //f Enabling Simplex Mode (Receive Only in this case)(SPI_RECEIVE_ONLY) - BIDIMODE must not be set (SPI_BIDIRECTIONAL)
  spi_lsb_first_t lsb_first;                    //Configures Endianess
  spi_crc_enable_t enable_crc;                  //Enable Cyclic Redundancy Check
  spi_crc_length_t crc_code_length;             //Configure the length of the Cyclic Redundancy Check Code (16 or 8 bits)
  spi_ssm_enable_t ssm_enabled;                 //Configure Software Slave Management - If enabled (SPI_SSM_ENABLED), the NSS pin input is replaced with the SSI bit value
  spi_ssi_bit_t ssi_bit;                        //The SSI bit for use in SSM mode, this bit will drive the NSS pin if SSM is enabled (.ssm_enabled = SPI_SSM_ENABLED)
  spi_master_selection_t master_selction;       //Configures as Master or Slave
  spi_ss_output_t slave_select_enabled;
  spi_nss_pulse_t nss_pulse;                    //If enabled (SPI_GENERATE_NSS_PULSE), NSS pulses between individual data transfers
  spi_fifo_rx_threshold_t fifo_rx_threshold;    //Looking into this more!
  spi_dma_rx_data_length_t dma_rx_data_length;  //Configure the length of the dma data transfer on the RX pin (select the parity)
  spi_dma_tx_data_length_t dma_tx_data_length;  //Configure the length of the dma data transfer on the TX pin (select the parity)
  spi_frame_format_t frame_format;              //Enumerates the Frame Format Mode (SPI Motorola vs SPI TI modes)
  uint8_t data_size_in_bits;                  //Configure the Data Length for the transfer (e.g. 4-bits, 5-bits, ..., 16-bits)
  uint16_t crc_polynomial;                      //Configure the CRC Polynomial
}spi_config_t;

typedef struct{
  const void* buffer;
  uint16_t buffer_size; 
  uint8_t data_size;
}spi_data_packet_t;
/*================ CONFIG FUNCTIONS ================*/

/**
 * @brief Configure the SPI peripheral
 * @param spi_line Pointer to the SPI peripheral to be configured
 * @param cfg Pointer to the configuration struct containing settings for the spi peripheral
 * 
 * @retval An Error Code of Format SPI_ERR_X or SPI_OK (if there was no error)
 * @note Doesn't Enable SPI
 */
int8_t spi_config(SPI_TypeDef* spi_line, const spi_config_t* cfg);
int8_t spi_enable(SPI_TypeDef* spi_line);
int8_t spi_disable(SPI_TypeDef* spi_line, spi_rxonly_t rxonly);
int8_t spi_disable_nonblocking(SPI_TypeDef* spi_line, uint8_t rxonly);
int8_t spi_is_enabled(const SPI_TypeDef* spi_line);
int8_t spi_assert_nss(SPI_TypeDef* spi_line);

/*============== TRANSMIT FUNCTIONS =================*/

int8_t spi_transmit_polling(SPI_TypeDef* spi_line, const spi_data_packet_t* data_packet);
int8_t spi_transfer_interrupts(SPI_TypeDef* spi_line, const spi_data_packet_t* data_packet);
int8_t spi_set_dmatxen(SPI_TypeDef* spi_line, uint8_t en);

/*============== RECEIVE FUNCTIONS =================*/

int8_t spi_receive_polling(SPI_TypeDef* spi_line, void* buffer, uint32_t length, uint8_t data_size);
int8_t spi_set_dmarxen(SPI_TypeDef* spi_line, uint8_t en);

/*============== ISR FUNCTIONS =================*/

void SPI1_IRQHandler();
void SPI3_IRQHandler();

/*==================== HELPERS ====================*/

/**
 * @brief Writes bits, or a bit, to a register. First clearing and then writing.
 * @param reg Pointer to the register where the bits will be written
 * @param mask Mask of the bits to be written - used to clear the bits
 * @param pos Position in the register to write the bits to
 * @param value The Bits to be written
 * 
 * @retval An error code or SPI_OK
 */
static inline int8_t write_bits(volatile uint32_t* reg, uint32_t  mask, uint32_t pos, uint8_t value){
  if(!reg)
    return SPI_ERR_INVALID_PARAM;
  
  *reg = (*reg & ~mask ) | ((value << pos) & mask);

  return SPI_OK;
}


#endif