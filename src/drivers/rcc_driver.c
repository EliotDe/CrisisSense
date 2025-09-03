#include "stm32l432xx.h"
#include "rcc_driver.h"

// Board Configuration
#ifndef HSE_CLK_HZ
#define HSE_CLK_HZ  8000000U
#endif 

#ifndef BOARD_HAS_HSE
#define BOARD_HAS_HSE 1       // set to 0 if there is none
#endif 

#define HSI16_CLK_HZ 16000000 //this is the typical value, could be between 15.88MHz and 16.08MHz


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
static size_t rcc_get_hse_hz(){
  #ifdef HSE_CLK_HZ
    // Return HSE frequency if the oscillator is ready
    if(RCC->CR & RCC_CR_HSERDY)
      return HSE_CLK_HZ;
  
  #endif

  return 0; // HSE not present or not ready
}

static size_t rcc_get_msi_hz(){
  if(!RCC->CR & RCC_CR_MSIRDY)
    return 0; // MSI not ready

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
size_t rcc_get_sysclk_hz(){
  uint8_t sysclk_src = RCC->CFGR & RCC_CFGR_SWS;

  if(sysclk_src == RCC_CFGR_SWS_HSE)
    return rcc_get_hse_hz();
  else if (sysclk_src == RCC_CFGR_SWS_HSI)
    return HSI16_CLK_HZ;
  else if (sysclk_src == RCC_CFGR_SWS_MSI)
    return rcc_get_msi_hz();
  else{ // If PLL is being used: Read PLLCFGR (PLL Configuration Register)
    size_t pll_input_clk_hz;
    switch(RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC){
      case RCC_PLLCFGR_PLLSRC_HSE: pll_input_clk_hz = rcc_get_hse_hz(); break;
      case RCC_PLLCFGR_PLLSRC_HSI: pll_input_clk_hz = HSI16_CLK_HZ; break;
      case RCC_PLLCFGR_PLLSRC_MSI: pll_input_clk_hz = rcc_get_msi_hz(); break;
      default: return 0; // Invalid PLL Source
    };

    return rcc_calculate_pll_output_hz(pll_input_clk_hz);
    // Calculate Output Frequency From Ref Manual Equations
  }

  // Read SWS - System Clock Switch Status
  return 0;
}

static size_t rcc_calculate_pll_output_hz(size_t pll_input_clk_hz){
  if (!pll_input_clk_hz) {
      return 0;
  }
  
  if (!(RCC->CR & RCC_CR_PLLRDY)) {
      return 0; // PLL not ready
  }
  uint8_t plln_bits = RCC->PLLCFGR & RCC_PLLCFGR_PLLN;
  uint8_t pllm_bits = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;
  uint8_t pllm = pllm_bits + 1; // register stores PLLM = 1 as 000, PLLM = 2 as 001, ...

  // uint8_t pllp_bits = RCC->PLLCFGR & RCC_PLLCFGR_PLLP;
  // uint8_t pllp = pllp_bits ? 17 : 7; // register stores PLLP = 7 as 0, PLLP = 17 as 1
  // uint8_t pllq_bits = RCC->PLLCFGR & RCC_PLLCFGR_PLLQ;
  // uint8_t pllq = 2 * (pllq_bits + 1); // register stores PLLQ = 2 as 00, PLLQ = 4 as 01, ...
  uint8_t pllr_bits = RCC->PLLCFGR & RCC_PLLCFGR_PLLR;
  uint8_t pllr = 2 * (pllr_bits + 1); // register stores PLLR = 2 as 00, PLLR = 4 as 01, ...

  size_t vco_output_hz = pll_input_clk_hz * (plln_bits / pllm);
  // size_t pllp_hz = vco_output_hz / pllp; 
  // size_t pllq_hz = vco_output_hz / pllq; 
  size_t pllr_hz = (size_t) (vco_output_hz / pllr); // for SYSCLK

  return pllr_hz; // placeholder
}

size_t rcc_get_hclk_hz(){
  // Read AHB Prescaler
  size_t sysclk_hz = rcc_get_sysclk_hz();
  uint8_t ahb_prescaler_bits = RCC->CFGR & RCC_CFGR_HPRE;

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
  
  return sysclk_hz / ahb_prescaler;
}

size_t  rcc_get_pclk1_hz(){
  // Read APB1 Prescaler 
  size_t hclk_hz = rcc_get_hclk_hz();
  uint8_t apb1_prescaler_bits = RCC->CFGR & RCC_CFGR_PPRE1;
  
  if ((apb1_prescaler_bits >> 3) == 0)
    return hclk_hz;
  
  static const uint8_t apb1_prescaler_value[] = {
    (uint8_t)2,
    (uint8_t)4,
    (uint8_t)8,
    (uint8_t)16
  };

  uint8_t apb1_prescaler = apb1_prescaler_value[apb1_prescaler_bits & APB_PRESCALER_Msk]; 
  return hclk_hz / apb1_prescaler;
}

size_t rcc_get_pclk2_hz(){
  // Read APB2 Prescaler
  // Read APB1 Prescaler 
  size_t hclk_hz = rcc_get_hclk_hz();
  uint8_t apb2_prescaler_bits = RCC->CFGR & RCC_CFGR_PPRE2;

  if ((apb2_prescaler_bits >> 3) == 0)
    return hclk_hz;

  static const uint8_t apb2_prescaler_value[] = {
    (uint8_t)2,
    (uint8_t)4,
    (uint8_t)8,
    (uint8_t)16
  };

  uint8_t apb2_prescaler = apb2_prescaler_value[apb2_prescaler_bits & APB_PRESCALER_Msk];
  
  return hclk_hz / apb2_prescaler;

}