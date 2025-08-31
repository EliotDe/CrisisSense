#include "debugging_manager.h"
#include "usart_driver.h"
#include "dma_driver.h"
#include "stm32l432xx.h"

/**
 * @brief Runs Debug Manager (blocking) - Essentially Uses USART to print a debug message
 * @param debug_msg The Message to transmit
 * @retval 0 - If there is an Error 1 - Otherwise
 */
uint8_t Manager_Debug_Polling(const char* debug_msg){
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
        .data_size = length_of_transfer,
        .dma_channel = DMA1_Channel7,
        .interrupts = DMA_INT_NONE, //placeholder
        .mem_increment_mode = DMA_MEMORY_INCREMENT_ENABLED,
        .periph_increment_mode = DMA_PERIPH_INCREMENT_DISABLED,
        .mem_size = DMA_MSIZE_8,
        .periph_size = DMA_PSIZE_8,
        .memory_address = mem_address,
        .periph_address = &USART2->TDR,
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
        .f_clk = (uint32_t)1, //placeholder
        .word_length = USART_WORD_LENGTH_8
        };

        uint8_t usart_config_retval = usart_config_line(&usart_cfg, &usart_cfg_error);
        if (!usart_config_retval){
            return 0;
        }

    } else if (usart_mode == USART_MODE_POLLING){
        usart_config_t usart_cfg = {
        .usart_line = USART2,
        .auto_baud = USART_AUTOBAUD_DISABLED,
        .baud_rate = (uint32_t)9600,
        .dma = USART_DMA_NONE,
        .oversampling = USART_OVER16,
        .parity = USART_PARITY_NONE,
        .stop_bits = USART_STOP_1,
        .f_clk = (uint32_t)1, //placeholder
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
    // Enable Clock for GPIO peripheral

    // Configure MODER
    // Configure AFR
    // Configure OTYPER
    // Configure PUPDR
    // Configure OSPEEDR
}

static uint8_t Manager_RCC_Config(void) {
    // Enable HSE and configure PLL if necessary
    // HSI - default
    // Adjust later
}