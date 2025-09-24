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
  uint8_t manager_retval;
  manager_retval = manager_init();

  
  // Every five minutes:
  char buffer[100];
  manager_read_sensor_data(COMPENSATE_SENSOR_DATA, buffer);
  Manager_Debug_Polling(buffer);

  if(manager_retval == 0){
    LED_Init();
  }

  while(1);
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

    for(volatile uint32_t i=0; i<100000; i++);
  }
}