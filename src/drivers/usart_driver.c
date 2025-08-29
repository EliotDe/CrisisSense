#ifdef UNIT_TEST
    #include "tests\mock_stm32l432xx.h"
#else
    #include "stm32l432xx.h"
#endif 

#include "usart_driver.h"
#include <stddef.h>

#define USART_DIV_MIN_VALUE 16
#define USART_DIV_MAX_VALUE 0xFFFFU

static uint8_t usart_config_baudrate(USART_TypeDef* usart_line, const usart_baud_config_t* baud_cfg, usart_err_t* error);
static uint8_t usart_config_dma(USART_TypeDef* usart_line, usart_dma_t dma, usart_dma_ddre_t dma_ddre, usart_err_t* error);

static uint8_t usart_puts(USART_TypeDef* usart_line, const uint16_t* buffer, size_t buffer_size, usart_word_length_t word_length, usart_err_t* error);
static uint8_t usart_putc(USART_TypeDef* usart_line, const uint16_t c, usart_word_length_t word_length, usart_err_t* error);
/*==========================CONFIGURATION CODE==========================*/

/**
 * @brief Configures a given USART peripheral
 * @param cfg Pointer to a struct containing config information: baud rate, word length, parity, interrupts, oversampling, DMA
 * @param error Pointer to an error - If an error occurs, the value at this address is changed to an error flag
 * @retval 0 - An error has occured 1 - otherwise 
 * 
 * @note If configuring for continuous communication with DMA - the DMA peripheral should be configured separately
 * @todo Triple-Check Safety
 */
uint8_t usart_config_line(usart_config_t* cfg, usart_err_t* error){
    /*============ VALIDATION ============*/
    
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
        cfg->word_length > USART_WORD_LENGTH_7 || 
        cfg-> auto_baud > USART_AUTOBAUD_55_FRAME ||
        cfg->dma > USART_DMA_RX_TX) {
        if (error) *error = USART_ERR_INVALID_PARAM;
        return 0;
    }

    /*============== READ ==============*/

    uint32_t cr1 = cfg->usart_line->CR1;
    uint32_t cr2 = cfg->usart_line->CR2;

    /*============= MODIFY =============*/

    // oversampling
    if (cfg->oversampling == USART_OVER16)
        usart_register_clear_bit(&cr1, USART_CR1_OVER8);
    else
        usart_register_set_bit(&cr1, USART_CR1_OVER8);

    // word length

    // Clear both M0 and M1 first
    usart_register_clear_bit(&cr1, USART_CR1_M0);
    usart_register_clear_bit(&cr1, USART_CR1_M1);

    if (cfg->word_length == USART_WORD_LENGTH_7) {
        usart_register_set_bit(&cr1, USART_CR1_M1);      // M1=1, M0=0
    } else if (cfg->word_length == USART_WORD_LENGTH_9) {
        usart_register_set_bit(&cr1, USART_CR1_M0);      // M1=0, M0=1
    }

    // Parity
    if (cfg->parity == USART_PARITY_NONE){
        usart_register_clear_bit(&cr1, USART_CR1_PCE);
    }
    else{
        usart_register_set_bit(&cr1, USART_CR1_PCE);
        if (cfg->parity == USART_PARITY_EVEN)
            usart_register_clear_bit(&cr1, USART_CR1_PS);
        else
            usart_register_set_bit(&cr1, USART_CR1_PS);
    }

    // Stop Bits
    usart_register_clear_bit(&cr2, USART_CR2_STOP_Msk);
    usart_register_set_bit(&cr2, cfg->stop_bits << USART_CR2_STOP_Pos);

    // Configure Baud Rate
    usart_err_t baud_error = USART_OK;
    usart_baud_config_t baud_cfg ={
        .baud_rate = cfg->baud_rate,
        .oversampling = cfg->oversampling,
        .f_clk = cfg -> f_clk
    };

    if (!usart_config_baudrate(cfg->usart_line, &baud_cfg, &baud_error)){
        if (error) *error = baud_error;
        return 0;
    }

    if (cfg->auto_baud != USART_AUTOBAUD_DISABLED){ 
        // Clear existing mode bits
        usart_register_clear_bit(&cr2, USART_CR2_ABRMODE_Msk);

        // Specific Mode is set (START_BIT is 0, so already cleared above)
        if (cfg->auto_baud != USART_AUTOBAUD_START_BIT){
            usart_register_set_bit(&cr2, cfg->auto_baud);
        }

        // Enable auto baud detection
        usart_register_set_bit(&cr2, USART_CR2_ABREN);
    }

    // Configure DMA 
    usart_err_t dma_error = USART_OK;
    if (cfg->dma != USART_DMA_NONE){ // If DMA is configured
        if (!usart_config_dma(cfg->usart_line, cfg->dma, cfg->dma_ddre, &dma_error)){
            if (error) *error = dma_error;
            return 0;
        }
    }
    else{
        uint32_t cr3 = cfg->usart_line->CR3;
        usart_register_clear_bit(&cr3, (USART_CR3_DMAR | USART_CR3_DMAT));
        cfg->usart_line->CR3 = cr3;
    }
    

    /*============ WRITE =============*/

    cfg->usart_line->CR1 = cr1;
    cfg->usart_line->CR2 = cr2;
    cfg->usart_line->CR1 |= USART_CR1_UE; // Enable usart

    return 1;
}

/**
 * @brief This function sets the value for the USART_BRR register by calculating the variable usart_div using the clock frequency and oversampling method
 * @param usart_line The USART peripheral on which the baud rate is being configured
 * @param baud_cfg Pointer to a struct containing the desired baud rate, oversampling method and clock frequency
 * @param error Pointer to the error variable 
 * 
 * @retval 0 - If an error occurred 1- If the function was successful
 */
static uint8_t usart_config_baudrate(USART_TypeDef* usart_line, const usart_baud_config_t* baud_cfg, usart_err_t* error){
    if (!usart_line || !baud_cfg->baud_rate || !baud_cfg->f_clk ||
        (baud_cfg->oversampling != USART_OVER16 && baud_cfg->oversampling != USART_OVER8)){

        if (error) *error = USART_ERR_INVALID_PARAM;
        return 0;
    }

    uint32_t factor = (baud_cfg->oversampling == USART_OVER8) ? 2U : 1U;  // USART_OVER8 = 1, USART_OVER16 = 0
    uint64_t usart_div = ((uint64_t)(factor * baud_cfg->f_clk)) / (uint64_t)baud_cfg->baud_rate;

    if(usart_div > USART_DIV_MAX_VALUE){
        if (error) *error = USART_ERR_OVERFLOW;
        return 0;
    }

    if(usart_div < USART_DIV_MIN_VALUE){
        if (error) *error = USART_ERR_INVALID_PARAM;
        return 0;
    }

    if (baud_cfg->oversampling == USART_OVER16){
        usart_line->BRR = (uint32_t)usart_div;
    }
    else{
        //OVER8 mode: BRR[2:0] = USARTDIV[3:0] >> 1, BRR[15:4] = USARTDIV[15:4]
        uint32_t brr = ((uint32_t)usart_div & USART_BRR_DIV_MANTISSA_Msk) | // Keep bits 15:4
                       (((uint32_t)usart_div & USART_BRR_DIV_FRACTION_Msk) >> 1); // Shift bits 3:0 right by 1 when in over8 mode as per the reference manual 
        
        usart_line->BRR = brr;
    }

    return 1;
}

/**
 * @brief Configures the dma bits in the relevant USART register (CR3)
 * @param usart_line USART peripheral to configure DMA on
 * @param dma DMA settings - USART_DMA_RX (dma received bytes), USART_DMA_TX (dma transmitted bytes), USART_DMARX_TX (both)
 * @param dma_ddre DMA Disable on Reception Error
 * 
 * @retval 0 - An Error Occured 1 - Otherwise
 */
static uint8_t usart_config_dma(USART_TypeDef* usart_line, usart_dma_t dma, usart_dma_ddre_t dma_ddre, usart_err_t* error){
    if (!usart_line){
        if (error) *error = USART_ERR_INVALID_PARAM;
        return 0;
    }

    uint32_t cr3 = usart_line->CR3;

    usart_register_clear_bit(&cr3, (USART_CR3_DMAR | USART_CR3_DMAT)); // clear dma bits first
    usart_register_set_bit(&cr3, dma);

    if (dma_ddre == USART_DMA_NOT_DISABLED_ON_ERROR){
        usart_register_clear_bit(&cr3, USART_CR3_DDRE);
    }
    else{
        usart_register_set_bit(&cr3, USART_CR3_DDRE);
    }

    usart_line->CR3 = cr3;
    
    return 1;
}

/*============ TRANSMITTER CODE ============*/

/**
 * @brief This function starts USART transmission.
 * @param usart_line The USART Peripheral to start transmission on.
 * 
 * @retval 0 - An Error Occured 1 - Transmission Started OK
 * @note If transmitting with DMA, this function just enables transmission TE - You need to wait for TC after calling this.
 */
uint8_t usart_transmit(USART_TypeDef* usart_line, const usart_dataPacket_t* data_packet, usart_err_t* error){
    // placeholder
    if (!usart_line || !data_packet){
        if(error) *error = USART_ERR_INVALID_PARAM;
        return 0;
    }

    if(usart_line->CR1 & USART_CR1_TE){ // Check if transmitter is already enabled
        if(error) *error = USART_ERR_BUSY;
        return 0;
    }

    usart_line->CR1 |= USART_CR1_TE;


    // Send Data
    usart_err_t puts_error = USART_OK;
    if(data_packet->mode == USART_MODE_POLLING){
        uint8_t puts_retval = usart_puts(usart_line, data_packet->buffer, data_packet->length, data_packet->word_length, &puts_error);
        if(!puts_retval){
            *error = puts_error;
            return 0;
        }
    }
    
    return 1;
}

/**
 * @note I can't imagine this accounts for different word lengths
 */
static uint8_t usart_putc(USART_TypeDef* usart_line, const uint16_t c, usart_word_length_t word_length, usart_err_t* error){
    if(!usart_line){
        if(error) *error = USART_ERR_INVALID_PARAM;
        return 0;
    }
    
    while(!(usart_line->ISR & USART_ISR_TXE));

    static const uint16_t word_length_max[] = {
        0xFF,   // USART_WORD_LENGTH_8
        0x1FF,  // USART_WORD_LENGTH_9
        0X7F    // USART_WORD_LENGTH_7
    };

    if (c > word_length_max[word_length]){
        if (error) *error = USART_ERR_OVERFLOW;
        return 0;
    }

    usart_line->TDR = c;
    return 1;
}

/**
 * @note I can't imagine this accounts for different word lengths
 * @note Blocking
 */
static uint8_t usart_puts(USART_TypeDef* usart_line, const uint16_t* buffer, size_t length, usart_word_length_t word_length, usart_err_t* error){
    // word_length > USART_WORD_LENGTH_7 seems contradictory because 7 is the smallest word length - maybe change this
    if(!buffer || !length || word_length > USART_WORD_LENGTH_7){
        if(error) *error = USART_ERR_INVALID_PARAM;
        while(!(usart_line->ISR & USART_ISR_TC));
        return 0;
    }

    size_t chars_transmitted = 0;
    usart_err_t putc_error = USART_OK;
    while (chars_transmitted != length){
        uint8_t putc_retval = usart_putc(usart_line, buffer[chars_transmitted++], word_length, &putc_error);
        if(!putc_retval){
            if (error) *error = putc_error;
            while(!(usart_line->ISR & USART_ISR_TC));
            return 0;
        }
    }

    while(!(usart_line->ISR & USART_ISR_TC));

    return 1;
}