#ifndef GPIO_DRIVER_H
#define GPIO_DRIVER_H

// GPIO Error Codes
typedef enum{
  GPIO_OK,
  GPIO_ERR_INVALID_PARAM,
}gpio_error_t;

#ifndef GPIO_MODER_Msk
#define  GPIO_MODER_Msk  0x3U
#endif

#ifndef GPIO_OTYPER_Msk
#define  GPIO_OTYPER_Msk  0x1U
#endif

#ifndef GPIO_OSPEEDR_Msk
#define  GPIO_OSPEEDR_Msk  0x3U
#endif

#ifndef GPIO_PUPDR_Msk
#define  GPIO_PUPDR_Msk  0x3U 
#endif

// GPIO Mode Register Values
typedef enum{
  GPIO_MODER_INPUT,
  GPIO_MODER_OUTPUT,
  GPIO_MODER_AF,
  GPIO_MODER_ANALOG
}gpio_moder_t;

// GPIO Output Type Register Values
typedef enum{
  GPIO_OTYPER_PUSH_PULL,
  GPIO_OTYPER_OPEN_DRAIN
}gpio_otyper_t;

// GPIO Output Speed Register Values
typedef enum{
  GPIO_OSPEEDR_LOW,
  GPIO_OSPEEDR_MEDIUM,
  GPIO_OSPEEDR_HIGH,
  GPIO_OSPEEDR_VERY_HIGH
}gpio_ospeedr_t;

// GPIO pull-up/pull-down Register Values
typedef enum{
  GPIO_PUPDR_NONE,
  GPIO_PUPDR_PULL_UP,
  GPIO_PUPDR_PULL_DOWN
}gpio_pupdr_t;


typedef struct{
  GPIO_TypeDef* gpio_peripheral;
  uint8_t gpio_pin;
  gpio_moder_t mode;
  gpio_otyper_t output_type;
  gpio_pupdr_t pupdr_config;
  gpio_ospeedr_t output_speed;
  uint8_t alternate_function_code;
}gpio_config_t;


/*=============CONFIGURATION FUNCTIONS=============*/
uint8_t gpio_config_pin(gpio_config_t* cfg, gpio_error_t* error);

/*==============CONFIGURATION HELPERS==============*/
static inline void gpio_register_set_bit(uint32_t* reg, uint32_t bit){*reg |= bit;}
static inline void gpio_register_clear_bit(uint32_t* reg, uint32_t bit){*reg &= ~bit;}

#endif