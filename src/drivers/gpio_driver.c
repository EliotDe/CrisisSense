/**
 * @brief GPIO driver for stm32l432
 * 
 * @todo Allow for configuration of multiple pins on one Peripheral
 * This would be useful for configuring GPIO for say SPI with pins MOSI, MISO, NSS, SCK
 */


#include "stm32l432xx.h"
#include "gpio_driver.h"

#define GPIO_PIN_MAX 15u
#define GPIO_MODER_SHIFT_WIDTH 2u
#define GPIO_OSPEEDR_SHIFT_WIDTH 2u
#define GPIO_PUPDR_SHIFT_WIDTH 2u
#define GPIO_AFR_SHIFT_WIDTH 4u
#define ALT_FUNCTION_CODE_MIN 0u
#define ALT_FUNCTION_CODE_MAX 15u
#define ALT_FUNCTION_CODE_Msk 0xFu
/**
 * @brief Configures a GPIO peripheral
 * @param cfg Pointer to a struct containing the configuration
 * @param error An error code to be returned, could be 0 (GPIO_OK)
 * 
 * @param retval 0 - An error has occured 1 - Otherwise
 */
uint8_t gpio_config_pin(gpio_config_t* cfg, gpio_error_t* error){
  if(!cfg || !cfg->gpio_peripheral){
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

  // Configure GPIO Mode (2 bits per pin)
  uint32_t gpio_moder = cfg->gpio_peripheral->MODER;
  gpio_register_clear_bit(&gpio_moder, (GPIO_MODER_Msk << (cfg->gpio_pin * GPIO_MODER_SHIFT_WIDTH))); // Clears MODER bits to 00, (GPIO_MODE_INPUT = 0)
  gpio_register_set_bit(&gpio_moder, cfg->mode << (cfg->gpio_pin * GPIO_MODER_SHIFT_WIDTH));

  cfg->gpio_peripheral->MODER = gpio_moder;

  // Confgiure the output type, if any
  uint32_t gpio_otyper = cfg->gpio_peripheral->OTYPER;
  gpio_register_clear_bit(&gpio_otyper, GPIO_OTYPER_Msk << cfg->gpio_pin);
  gpio_register_set_bit(&gpio_otyper, cfg->output_type << cfg->gpio_pin);

  cfg->gpio_peripheral->OTYPER = gpio_otyper;

  // Configure Ouput Speed
  uint32_t gpio_ospeedr = cfg->gpio_peripheral->OSPEEDR;
  gpio_register_clear_bit(&gpio_ospeedr, (GPIO_OSPEEDR_Msk << (cfg->gpio_pin * GPIO_OSPEEDR_SHIFT_WIDTH)));
  gpio_register_set_bit(&gpio_ospeedr, (cfg->output_speed << (cfg->gpio_pin * GPIO_OSPEEDR_SHIFT_WIDTH)));
  
  cfg->gpio_peripheral->OSPEEDR = gpio_ospeedr;

  // Configure Pull-up / Pull-down on the GPIO line
  uint32_t gpio_pupdr = cfg->gpio_peripheral->PUPDR;
  gpio_register_clear_bit(&gpio_pupdr, (GPIO_PUPDR_Msk << (cfg->gpio_pin * GPIO_PUPDR_SHIFT_WIDTH)));
  gpio_register_set_bit(&gpio_pupdr, (cfg->pupdr_config << (cfg->gpio_pin * GPIO_PUPDR_SHIFT_WIDTH)));
  
  cfg->gpio_peripheral->PUPDR = gpio_pupdr;

  // Configure an Alternate Function

  if(cfg->mode == GPIO_MODER_AF){
    uint8_t afr_index = ((uint32_t)cfg->gpio_pin / 8u); // Each AFR register has 8 programmable values each being a nibble long (AFRL configures pins 0-7, AFRH configures pins 8-15)
    uint8_t afr_pin = ((uint32_t)cfg->gpio_pin % 8u);
    uint32_t gpio_afr = cfg->gpio_peripheral->AFR[afr_index];
    gpio_register_clear_bit(&gpio_afr, (ALT_FUNCTION_CODE_Msk << (afr_pin * GPIO_AFR_SHIFT_WIDTH)));
    gpio_register_set_bit(&gpio_afr, ((uint32_t)cfg->alternate_function_code << (afr_pin * GPIO_AFR_SHIFT_WIDTH)));

    cfg->gpio_peripheral->AFR[afr_index] = gpio_afr;
  }
  

  return 1;
}