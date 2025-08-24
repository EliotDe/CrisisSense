// PLACEHOLDER

#include "usart_driver.h"
#include "stm32l432xx.h"

/**
 * @brief Configures a given USART peripheral
 * @param cfg Pointer to a struct containing config information: baud rate, word length, parity, interrupts, oversampling, DMA
 * @param error Pointer to an error - If an error occurs, the value at this address is changed to an error flag
 * @retval 0 - An error has occured 1 - otherwise 
 * 
 * @note If configuring for continuous communication with DMA - the DMA peripheral should be configured separately
 */
uint8_t usart_config_line(usart_config_t* cfg, usart_err_t* error){
    if (!cfg || !cfg->baud_rate){ // cfg and baud-rate are the only necessary paramaters anything else can be set to a default
        if (error) *error = USART_ERR_INVALID_PARAM;
        return 0;
    }

    if (cfg->usart_line->CR1 & USART_CR1_UE){// If USART is enabled
        if(error) *error = USART_ERR_BUSY;
        return 0;
    }

    // Range validation
    if (cfg->stop_bits > USART_STOP_1_HALF || 
        cfg->parity > USART_PARITY_ODD ||
        cfg->word_length > USART_WORD_LENGTH_7) {
        if (error) *error = USART_ERR_INVALID_PARAM;
        return 0;
    }

    uint32_t cr1 = cfg->usart_line->CR1;
    uint32_t cr2 = cfg->usart_line->CR2;

    // oversampling
    if (cfg->oversampling == USART_OVER16)
        usart_cr1_clear_bit(&cr1, USART_CR1_OVER8);
    else
        usart_cr1_set_bit(&cr1, USART_CR1_OVER8);

    // word length

    // Clear both M0 and M1 first
    usart_cr1_clear_bit(&cr1, USART_CR1_M0);
    usart_cr1_clear_bit(&cr1, USART_CR1_M1);

    if (cfg->word_length == USART_WORD_LENGTH_7) {
        usart_cr1_set_bit(&cr1, USART_CR1_M1);      // M1=1, M0=0
    } else if (cfg->word_length == USART_WORD_LENGTH_9) {
        usart_cr1_set_bit(&cr1, USART_CR1_M0);      // M1=0, M0=1
    }

    // Parity
    if (cfg->parity == USART_PARITY_NONE){
        usart_cr1_clear_bit(&cr1, USART_CR1_PCE);
    }
    else{
        usart_cr1_set_bit(&cr1, USART_CR1_PCE);
        if (cfg->parity == USART_PARITY_EVEN)
            usart_cr1_clear_bit(&cr1, USART_CR1_PS);
        else
            usart_cr1_set_bit(&cr1, USART_CR1_PS);
    }

    // Stop Bits
    usart_cr2_clear_bit(&cr2, USART_CR2_STOP_Msk);
    usart_cr2_set_bit(&cr2, cfg->stop_bits << USART_CR2_STOP_Pos);

    // Configure Baud Rate
    usart_err_t baud_error = USART_OK;
    if (!usart_config_baudrate(cfg->usart_line, cfg->baud_rate, cfg->oversampling, &baud_error)){
        if (error) *error = baud_error;
        return 0;
    }
    // Configure DMA 
    usart_err_t dma_error = USART_OK;
    if (!usart_config_dma(cfg->usart_line, &dma_error)){
        if (error) *error = dma_error;
        return 0;
    }

    cfg->usart_line->CR1 = cr1;
    cfg->usart_line->CR2 = cr2;
    cfg->usart_line->CR1 |= USART_CR1_UE; // Enable usart

    return 1;
}

uint8_t usart_config_baudrate(USART_TypeDef* usart_line, uint32_t baud_rate, usart_oversampling_t over8, usart_err_t* error){
    if (!usart_line || !baud_rate || (over8 != USART_OVER16 && over8 != USART_OVER8)){
        if (error) *error = USART_ERR_INVALID_PARAM;
        return 0;
    }

    // placeholder

    return 1;
}

uint8_t usart_config_dma(USART_TypeDef* usart_line, usart_err_t* error){
    if (!usart_line){
        if (error) *error = USART_ERR_INVALID_PARAM;
        return 0;
    }

    // placeholder
    
    return 1;
}