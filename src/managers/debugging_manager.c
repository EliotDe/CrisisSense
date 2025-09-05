/**
 * @todo Make Manager_Debug_DMA fully asynchronous, it's currently blocking
 *        - Configure DMA & USART
 *        - Start DMA transfer
 *        - Let DMA hardware automatically push bytes to USART
 *        - Trigger an ISR when the transfer is complete
 *        - In the ISR, call a callback function provided by the higher level application
 * 
 * @todo USART remains the same, the DMA driver needs an interrupt that get triggered on TC
 *       This Manager configures DMA + USART, stores callback, starts transfer
 * 
 * @todo Application Code needs to provide the callback function
 * 
 * @todo Consider altering string implementation, possibly cache repeated strings
 *       Consider a struct with length and string
 *       If expanding consider stack-based formatting
 */

#include "debugging_manager.h"
#include "usart_driver.h"
#include "dma_driver.h"
#include "rcc_driver.h"
#include "gpio_driver.h"
#include "stm32l432xx.h"
// #include <string.h>

#define USART2_TX_AF 7u

static uint8_t Manager_DMA_Config(const char* mem_address, size_t length_of_transfer);
static uint8_t Manager_USART_Config(usart_mode_t usart_mode);
static uint8_t Manager_GPIO_Config(void);
static uint8_t Manager_RCC_Config(void);
static size_t strlen(const char* s);

/*
usart_config_t usart_cfg = { ...common fields... };
usart_cfg.dma = (usart_mode == USART_MODE_DMA) ? USART_DMA_TX : USART_DMA_NONE;
*/


/**
 * @brief Runs Debug Manager (blocking) - Essentially Uses USART to print a debug message
 * @param debug_msg The Message to transmit
 * @retval 0 - If there is an Error 1 - Otherwise
 */
uint8_t Manager_Debug_Polling(const char* debug_msg){
  if(!debug_msg){
    return 0;
  }
  Manager_RCC_Config();
  Manager_GPIO_Config();
  uint8_t usart_config_retval = Manager_USART_Config(USART_MODE_POLLING);
  if(!usart_config_retval){
    return 0;
  }

  usart_dataPacket_t data_packet = {
    .buffer = debug_msg,
    .length = strlen(debug_msg),
    .word_length = USART_WORD_LENGTH_8,
    .mode = USART_MODE_POLLING
  };

  usart_err_t transmit_error = USART_OK;
  uint8_t transmit_retval = usart_transmit(USART2, &data_packet, &transmit_error);
  if(!transmit_retval){
    return 0;
  }

  return 1;
}

/**
 * @brief Runs Debug Manager with DMA - Uses DMA for larger debug messages
 * @param mem_address The address in memory that DMA will transfer data from
 * @retval 0 - An Error Has Occured  1 - Otherwise
 * 
 * @note Although devices support 9-bit words in USART, This application does not.
 *       For this reason I pass the address as a pointer to a character (8-bits) buffer.
 *       If needed change to a const void* type and then to uint16_t or uint8_t depending
 *       on word-length. Though for this application I see no reason why you would.
 */
uint8_t Manager_Debug_DMA(const char* mem_address, size_t length_of_transfer){
  if(!mem_address){
    return 0;
  }
  Manager_RCC_Config();
  Manager_GPIO_Config();
  uint8_t dma_config_retval = Manager_DMA_Config(mem_address, length_of_transfer);
  if (!dma_config_retval){
    return 0;
  }
  uint8_t usart_config_retval = Manager_USART_Config(USART_MODE_DMA);
  if(!usart_config_retval){
    return 0;
  }

  usart_dataPacket_t data_packet = {
    .buffer = mem_address, 
    .length = length_of_transfer,
    .word_length = USART_WORD_LENGTH_8,
    .mode = USART_MODE_DMA
  };

  usart_err_t usart_transmit_error = USART_OK;
  uint8_t usart_transmit_retval = usart_transmit(USART2, &data_packet, &usart_transmit_error);
  if(!usart_transmit_retval){
    while(!(USART2->ISR & USART_ISR_TC));
    return 0;
  }
  while(!(USART2->ISR & USART_ISR_TC));

  return 1;
}

static uint8_t Manager_DMA_Config(const char* mem_address, size_t length_of_transfer){
  if(!mem_address){
    return 0;
  }
  // validate data_size fits in a 16-bit register
  if(length_of_transfer == 0 || length_of_transfer > 0xFFFF){
    return 0;
  }

  dma_channel_config_t dma_config = {
    .channel_priority = DMA_PRIORITY_LOW,
    .circular_mode = DMA_CIRCULAR_DISABLED,
    .data_size = (uint16_t)length_of_transfer,
    .dma_channel = DMA1_Channel7,
    .interrupts = DMA_INT_NONE, //placeholder
    .mem_increment_mode = DMA_MEMORY_INCREMENT_ENABLED,
    .periph_increment_mode = DMA_PERIPH_INCREMENT_DISABLED,
    .mem_size = DMA_MSIZE_8,
    .periph_size = DMA_PSIZE_8,
    .memory_address = mem_address,
    .periph_address = (const void*)&USART2->TDR,
    .transfer_direction = DMA_READ_FROM_MEMORY
  };

  dma_err_t dma_error = DMA_OK;
  uint8_t dma_config_retval = dma_config_channel(&dma_config, &dma_error);
  if (!dma_config_retval){
    return 0;
  }

  return 1;
}

static uint8_t Manager_USART_Config(usart_mode_t usart_mode){
  usart_err_t usart_cfg_error = USART_OK;
  rcc_err_t rcc_error = RCC_OK;
  uint32_t pclk2_hz = rcc_get_pclk2_hz(&rcc_error);
  if (!pclk2_hz){
    return 0;
  }
  if (usart_mode == USART_MODE_DMA){
    usart_config_t usart_cfg = {
    .usart_line = USART2,
    .auto_baud = USART_AUTOBAUD_DISABLED,
    .baud_rate = (uint32_t)9600,
    .dma = USART_DMA_TX,
    .dma_ddre = USART_DMA_NOT_DISABLED_ON_ERROR,
    .oversampling = USART_OVER16,
    .parity = USART_PARITY_NONE,
    .stop_bits = USART_STOP_1,
    .f_clk = pclk2_hz, 
    .word_length = USART_WORD_LENGTH_8
    };

    uint8_t usart_config_retval = usart_config_line(&usart_cfg, &usart_cfg_error);
    if (!usart_config_retval){
      return 0;
    }
  }else if (usart_mode == USART_MODE_POLLING){
    usart_config_t usart_cfg = {
    .usart_line = USART2,
    .auto_baud = USART_AUTOBAUD_DISABLED,
    .baud_rate = (uint32_t)9600,
    .dma = USART_DMA_NONE,
    .oversampling = USART_OVER16,
    .parity = USART_PARITY_NONE,
    .stop_bits = USART_STOP_1,
    .f_clk = (uint32_t)rcc_get_pclk2_hz(), //placeholder
    .word_length = USART_WORD_LENGTH_8
    };

    uint8_t usart_config_retval = usart_config_line(&usart_cfg, &usart_cfg_error);
    if (!usart_config_retval){
      return 0;
    }
  }
  
  return 1;
}

static uint8_t Manager_GPIO_Config(void) {
  // Configure PA0 for USART_TX
  gpio_config_t cfg = {
    .gpio_peripheral = GPIOA,
    .gpio_pin = 2,
    .mode = GPIO_MODER_AF,
    .output_speed = GPIO_OSPEEDR_VERY_HIGH,
    .output_type = GPIO_OTYPER_PUSH_PULL,
    .pupdr_config = GPIO_PUPDR_NONE,
    .alternate_function_code = USART2_TX_AF       // AF7
  };

  gpio_error_t gpio_err = GPIO_OK;
  uint8_t gpio_config_retval = gpio_config_pin(&cfg, &gpio_err);
  if (!gpio_config_retval)
    return 0;

  return 1;
}

static uint8_t Manager_RCC_Config(void) {
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;    //GPIOA Clock
  RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN; //USART2 Clock
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;     //DMA1 Clock
    // Enable HSE and configure PLL if necessary, higher speeds!!!
    // HSI - default 16MHz, ok for baud rate = 9600
    // Adjust later
  return 1;
}

// TODO: Move to utils.c  -- I use the flag -nostdlib so i need this
static size_t strlen(const char* s){
  const char* p = s;
  while (*p) {
    p++;
  }
  return (size_t)(p-s);
}