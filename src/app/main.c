#include "stm32l432xx.h"

#define OUTPUT_MODE 1

#define TOGGLE_BIT(REG, BIT)    ((REG) ^= (1U << BIT))

#define BLINK_DELAY 1000000

void delay(volatile uint32_t d);

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
    GPIOB -> MODER &= ~GPIO_MODER_MODE3;
    /* set the mode to general purpose output */
    GPIOB -> MODER |= (OUTPUT_MODE << GPIO_MODER_MODE3_Pos);

    // ten blinks
    for(i=0; i < 10; ++i){
        TOGGLE_BIT(GPIOA -> ODR, 12);
        delay(BLINK_DELAY);
    }

    /* prevent undefined behaviour after return from main */
    while(1);
}

void delay(volatile uint32_t d){
    while (d--)
        __NOP();
}
