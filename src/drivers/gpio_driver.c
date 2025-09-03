#include "stm32l432xx.h"
#include "gpio_driver.h"

#define GPIO_PIN_MAX 15
#define ALT_FUNCTION_CODE_MIN 0
#define ALT_FUNCTION_CODE_MAX 15
#define ALT_FUNCTION_CODE_Msk 15
/**
 * @brief Configures a GPIO peripheral
 * @param cfg Pointer to a struct containing the configuration
 * @param error An error code to be returned, could be 0 (GPIO_OK)
 * 
 * @param retval 0 - An error has occured 1 - Otherwise
 */
uint8_t gpio_config_pin(gpio_config_t* cfg, gpio_error_t* error){
  if(!cfg->gpio_peripheral || !cfg->gpio_pin){
    if (error) *error = GPIO_ERR_INVALID_PARAM;
    return 0;
  }
  if(cfg->mode > GPIO_MODER_ANALOG ||
     cfg->output_type > GPIO_OTYPER_OPEN_DRAIN ||
     cfg->output_speed > GPIO_OSPEEDR_VERY_HIGH ||
     cfg->pupdr_config > GPIO_PUPDR_PULL_DOWN ||
     cfg->gpio_pin > GPIO_PIN_MAX || 
     cfg->alternate_function_code > ALT_FUNCTION_CODE_MAX){

    if (error) * error = GPIO_ERR_INVALID_PARAM;
    return 0;
  }

  // Configure GPIO Mode
  uint32_t gpio_moder = cfg->gpio_peripheral->MODER;
  gpio_register_clear_bit(&gpio_moder, GPIO_MODER_Msk << cfg->gpio_pin*2); // Clears MODER bits to 00, (GPIO_MODE_INPUT = 0)
  if (cfg->mode != GPIO_MODER_INPUT) 
    gpio_register_set_bit(&gpio_moder, cfg->mode << cfg->gpio_pin*2); // Two bits represent mode, so you need to multiply by 2 to get the position of the bits in the register

  cfg->gpio_peripheral->MODER = gpio_moder;

  // Confgiure the output type, if any
  uint32_t gpio_otyper = cfg->gpio_peripheral->OTYPER;
  gpio_register_clear_bit(&gpio_otyper, GPIO_OTYPER_Msk << cfg->gpio_pin);
  if (cfg->mode != GPIO_OTYPER_OPEN_DRAIN)
    gpio_register_set_bit(&gpio_otyper, cfg->output_type << cfg->gpio_pin);

  cfg->gpio_peripheral->OTYPER = gpio_otyper;

  // Configure Pull-up / Pull-down on the GPIO line
  uint32_t gpio_pupdr = cfg->gpio_peripheral->PUPDR;
  gpio_register_clear_bit(&gpio_pupdr, GPIO_PUPDR_Msk << cfg->gpio_pin*2);
  if (cfg->mode != GPIO_PUPDR_NONE)
    gpio_register_set_bit(&gpio_pupdr, cfg->pupdr_config << cfg->gpio_pin*2);
  
  cfg->gpio_peripheral->PUPDR = gpio_pupdr;

  // Configure an Alternate Function
  uint8_t afr_register = cfg->gpio_pin / 8; // Each AFR register has 8 programmable values each being a nibble long (AFRL configures pins 0-7, AFRH configures pins 8-15)
  uint8_t afr_pin = afr_register ? (cfg->gpio_pin-8) : (cfg->gpio_pin);
  uint32_t gpio_afr = cfg->gpio_peripheral->AFR[afr_register];
  gpio_register_clear_bit(&gpio_afr, ALT_FUNCTION_CODE_Msk << afr_pin*4);
  if (cfg->alternate_function_code != ALT_FUNCTION_CODE_MIN)
    gpio_register_set_bit(&gpio_afr, cfg->alternate_function_code << afr_pin*4);

  cfg->gpio_peripheral->AFR[afr_register] = gpio_afr;

  return 1;
}