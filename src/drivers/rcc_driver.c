#include "stm32l432xx.h"
#include "rcc_driver.h"

// Board Configuration
#ifndef HSE_CLK_HZ
#define HSE_CLK_HZ  8000000U
#endif 

// #ifndef BOARD_HAS_HSE
// #define BOARD_HAS_HSE 1       // set to 0 if there is none
// #endif 

#define HSI16_CLK_HZ 16000000U //this is the typical value, could be between 15.88MHz and 16.08MHz


#define AHB_PRESCALER_Msk 0x7U
#define APB_PRESCALER_Msk 0x3U


// HSI frequency lookup table - could differ depending on HSI or PLL mode
static const uint32_t msi_frequency_table[] = {
  100000,   // Range 0: 100 KHz
  200000,   // Range 1: 200 KHz
  400000,   // Range 2: 400 KHz
  800000,   // Range 3: 800 KHz
  1000000,  // Range 4: 1 MHz
  2000000,  // Range 5: 2 MHz
  4000000,  // Range 6: 4 MHz
  8000000,  // Range 7: 8 MHz
  16000000, // Range 8: 16 MHz
  24000000, // Range 9: 24 MHz
  32000000, // Range 10: 32 MHz
  48000000, // Range 11: 48 MHz
};

/**
 * @brief Return the frequency of the HSE external oscillator, if present on the board and ready
 */
static uint32_t rcc_get_hse_hz(rcc_err_t* error){
  #ifdef HSE_CLK_HZ
    // Return HSE frequency if the oscillator is ready
    if(RCC->CR & RCC_CR_HSERDY)
      return HSE_CLK_HZ;
  
  #endif

  if(error) *error = RCC_ERR_HSE_NOT_AVAILABLE;
  return 0; // HSE not present or not ready
}

static uint32_t rcc_get_msi_hz(rcc_err_t* error){
  if(!(RCC->CR & RCC_CR_MSIRDY)){
    if(error) *error = RCC_ERR_MSI_NOT_READY;
    return 0; // MSI not ready
  }

  uint32_t msirange;

  if (RCC->CR & RCC_CR_MSIRGSEL) // MSI range determined by CR register
    msirange = (RCC->CR & RCC_CR_MSIRANGE_Msk) >> RCC_CR_MSIRANGE_Pos;
  else                           // MSI range determined by CSR register
    msirange = (RCC->CSR & RCC_CSR_MSISRANGE_Msk) >> RCC_CSR_MSISRANGE_Pos;

  
  if (msirange < (sizeof(msi_frequency_table) / sizeof(msi_frequency_table[0])))
    return msi_frequency_table[msirange];
  
  return 4000000; //default value
}


/**
 * @brief Calculates the System Clock Frequency in Hertz (Hz)
 * @retval System Clock Frequency
 */
uint32_t rcc_get_sysclk_hz(rcc_err_t* error){
  uint8_t sysclk_src = RCC->CFGR & RCC_CFGR_SWS;

  switch(sysclk_src){
    case RCC_CFGR_SWS_HSE:{ // If the SYSCLK Source is configured as HSE
      // Get HSE frequency
      rcc_err_t hse_error = RCC_OK;
      uint32_t get_hse_retval = rcc_get_hse_hz(&hse_error);
      // If an error Occured Report it
      if(!get_hse_retval){
        if(error) *error = hse_error;
        return 0;
      }
      return get_hse_retval;
    }
    case RCC_CFGR_SWS_HSI: { // If the SYSCLK Source is configured as HSI
      // HSI should be 16MHz so just return that
      return HSI16_CLK_HZ;
    }
    case RCC_CFGR_SWS_MSI: { // If the SYSCLK Source is configured as MSI
      // Get MSI frequency
      rcc_err_t msi_error = RCC_OK;
      uint32_t get_msi_retval = rcc_get_msi_hz(&msi_error);
      // If an error occured report it
      if(!get_msi_retval){
        if(error) *error = msi_error;
        return 0;
      }
      return get_msi_retval;
    }
    case RCC_CFGR_SWS_PLL: { // If the SYSCLK Source is configured as PLL
      size_t pll_input_clk_hz;
      uint8_t pll_clk_src = RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC;
      switch(pll_clk_src){ 
        case RCC_PLLCFGR_PLLSRC_HSE: { // If the PLL Source is configured as HSE
          // Get HSE frequency
          rcc_err_t hse_error = RCC_OK;
          uint32_t get_hse_retval = rcc_get_hse_hz(&hse_error);
          // If an error occured, report it
          if(!get_hse_retval){
            if(error) *error = hse_error;
            return 0;
          }
          // Set the PLL input clock frequency to HSE clock frequency
          pll_input_clk_hz = get_hse_retval;
          break;
        }
        case RCC_PLLCFGR_PLLSRC_HSI: { // If the PLL Source is configured as HSI
          // Set the PLL input clock frequency to HSI clock frequency
          pll_input_clk_hz = HSI16_CLK_HZ; 
          break;
        }
        case RCC_PLLCFGR_PLLSRC_MSI: { // If the PLL Source is configured as MSI
          // Get MSI Frequency
          rcc_err_t msi_error = RCC_OK;
          uint32_t get_msi_retval = rcc_get_msi_hz(&msi_error);
          // If an error occured, report it
          if(!get_msi_retval){
            if(error) *error = msi_error;
            return 0;
          }
          // Set the PLL input clock frequency to MSI clock frequency
          pll_input_clk_hz = get_msi_retval;
          break;
        }
        default: { // If the PLL input clock is not recognised - report error and return 0
          if(error) *error = RCC_ERR_INVALID_PLL_SRC;
          return 0; // Invalid PLL Source
        } 
      };

      return rcc_calculate_pll_output_hz(pll_input_clk_hz);
    }
    default:{ // If the SYSCLK input is not recognised - report error and return 0
      if(error) *error = RCC_ERR_INVALID_SYSCLK_SRC;
      return 0;
    }
  }

  return 0; // If we've made it here something has gone horribly wrong
}

/**
 * @brief Calculate the frequency of the PLL output
 * @param pll_input_clk_hz Frequency of the clock inputted to PLL
 * @retval The frequency
 */
static uint32_t rcc_calculate_pll_output_hz(size_t pll_input_clk_hz){
  if (!pll_input_clk_hz) {
      return 0;
  }
  
  if (!(RCC->CR & RCC_CR_PLLRDY)) {
      return 0; // PLL not ready
  }
  uint32_t plln = (RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos;
  uint32_t pllm_bits = (RCC->PLLCFGR & RCC_PLLCFGR_PLLM) >> RCC_PLLCFGR_PLLM_Pos;
  uint32_t pllm = pllm_bits + 1u; // register stores PLLM = 1 as 000, PLLM = 2 as 001, ...

  uint32_t pllr_bits = (RCC->PLLCFGR & RCC_PLLCFGR_PLLR) >> RCC_PLLCFGR_PLLR_Pos;
  uint32_t pllr = 2u * (pllr_bits + 1u); // register stores PLLR = 2 as 00, PLLR = 4 as 01, ...

  uint64_t vco_output_hz = ((uint64_t)pll_input_clk_hz / pllm) * plln;
  uint32_t pllr_hz = (uint32_t)(vco_output_hz / pllr); // for SYSCLK

  return pllr_hz; // placeholder
}

/**
 * @brief Get the frequency of the H-Clock (used for AHB peripherals).
 * @param error Pointer to an error code - will be updated if an error occurs.
 * @retval Frequency of H-Clock, 0 - An error has occurred.
 */
uint32_t rcc_get_hclk_hz(rcc_err_t* error){
  // Get the frequncy of the system clock
  rcc_err_t sysclk_err = RCC_OK;
  uint32_t sysclk_hz = rcc_get_sysclk_hz(&sysclk_err);
  // If an error occured report it
  if(!sysclk_hz){
    if(error) *error = sysclk_err;
    return 0;
  }

  // Get AHB prescaler values
  uint8_t ahb_prescaler_bits = (RCC->CFGR & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos;

  if ((ahb_prescaler_bits >> 3) == 0) // as per reference manual, if HPRE[3:0] = 0xxx SYSCLK is not divided
    return sysclk_hz;

  static const uint16_t ahb_prescaler_value[] = {
    (uint16_t)2,
    (uint16_t)4,
    (uint16_t)8,
    (uint16_t)16,
    (uint16_t)64,
    (uint16_t)128,
    (uint16_t)256,
    (uint16_t)512
  };

  uint16_t ahb_prescaler = ahb_prescaler_value[ahb_prescaler_bits & AHB_PRESCALER_Msk]; // 
  // Validate sysclk_hz doesn't exceed maximum allowed frequency: Section 5.1.7: Dynamic Voltage Scaling Management
  
  return (uint32_t)sysclk_hz / ahb_prescaler;
}

uint32_t rcc_get_pclk1_hz(rcc_err_t* error){
  // Get the H-Clock Frequency
  rcc_err_t hclk_err = RCC_OK;
  uint32_t hclk_hz = rcc_get_hclk_hz(&hclk_err);
  // If an error occurred, report it
  if (!hclk_hz){
    if(error) *error = hclk_err;
    return 0;
  }
  
  // Get APB1 prescaler value
  uint8_t apb1_prescaler_bits = (RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos;
  
  if ((apb1_prescaler_bits >> 2) == 0)
    return hclk_hz;
  
  static const uint8_t apb1_prescaler_value[] = {
    (uint8_t)2,
    (uint8_t)4,
    (uint8_t)8,
    (uint8_t)16
  };

  uint8_t apb1_prescaler = apb1_prescaler_value[apb1_prescaler_bits & APB_PRESCALER_Msk]; 
  return (uint32_t)hclk_hz / apb1_prescaler;
}

uint32_t rcc_get_pclk2_hz(rcc_err_t* error){
  // Get the H-Clock Frequency
  rcc_err_t hclk_err = RCC_OK;
  uint32_t hclk_hz = rcc_get_hclk_hz(&hclk_err);
  // If an error occurred, report it
  if (!hclk_hz){
    if(error) *error = hclk_err;
    return 0;
  }
  uint8_t apb2_prescaler_bits = (RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos;

  if ((apb2_prescaler_bits >> 2) == 0)
    return hclk_hz;

  static const uint8_t apb2_prescaler_value[] = {
    (uint8_t)2,
    (uint8_t)4,
    (uint8_t)8,
    (uint8_t)16
  };

  uint8_t apb2_prescaler = apb2_prescaler_value[apb2_prescaler_bits & APB_PRESCALER_Msk];
  
  return (uint32_t)hclk_hz / apb2_prescaler;
}