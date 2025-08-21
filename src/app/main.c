#include "stm32l432xx.h"

#define LED_PIN 3
#define LED_PORT GPIOB 

void main(){
    /*//\\ //\\ //\\ //\\ //\\ //\\ //\\ //\\ //\\ //\\*/
    /*\\// \\// \\// \\// \\// \\// \\// \\// \\// \\//*/
    /*//\\ //\\  Theres nothing here yet //\\ //\\ //\\ -- there is now*/
    /*\\// \\// \\// \\// \\// \\// \\// \\// \\// \\//*/
    /*//\\ //\\ //\\ //\\ //\\ //\\ //\\ //\\ //\\ //\\*/
    int i;

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