#include "stm32l432xx.h"
#include "src/managers/debugging_manager.h"
#include "src/managers/sensor_manager.h"
#include "spi_driver.h"
#include "dma_driver.h"
#include "rcc_driver.h"
#include "gpio_driver.h"

#include <stdio.h>

#define LED_PIN 3U
#define LED_PORT GPIOB 

#define COMPENSATE_SENSOR_DATA 1U

static void LED_Init();

void main(){
  Manager_RCC_Config();
  Manager_GPIO_Config();
  uint8_t usart_config_retval = Manager_USART_Config(USART_MODE_POLLING);

  if(!usart_config_retval){
    LED_Init();
    while(1);
  }
  
  // cppcheck-suppress variableScope
  uint8_t manager_retval, read_retval;
  
  manager_retval = sensor_manager_init();
  if (manager_retval != SENSOR_OK) LED_Init();

  // Every five minutes:
  while(1){
    // Re-write at the top of the terminal
    Manager_Debug_Polling("\033[H");
    char buffer[100];
    for(uint8_t i = 0; i < 100; i++){
      buffer[i] = 0;
    }
    read_retval = manager_read_sensor_data(COMPENSATE_SENSOR_DATA, buffer);

    if (read_retval == SENSOR_OK)
      Manager_Debug_Polling(buffer);

    // Move away from this delay function
    user_delay_us(1000000, SPI1);
  }
}


static void LED_Init(){
  /* configure clock for AHB1 bus */
  RCC -> AHB2ENR |= RCC_AHB2ENR_GPIOBEN;

  /* clear the value for the green led in moder register */
  LED_PORT -> MODER &= ~GPIO_MODER_MODE3;
  /* set the mode to general purpose output */
  LED_PORT -> MODER |= (0x1 << GPIO_MODER_MODE3_Pos);

  LED_PORT -> OTYPER &= ~(1 << LED_PIN);          // Push-pull
  LED_PORT -> OSPEEDR &= ~(0x3 << (LED_PIN * 2)); // Low speed
  LED_PORT -> PUPDR &= ~(0x3 << (LED_PIN * 2));   // No pull-up/pull-down


  /* prevent undefined behaviour after return from main */
  while(1){
    LED_PORT -> ODR ^= (1 << LED_PIN);
    user_delay_us(1000000, SPI1);
    //for(volatile uint32_t i=0; i<100000; i++);
  }
}