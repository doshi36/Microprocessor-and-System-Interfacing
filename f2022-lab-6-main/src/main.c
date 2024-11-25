/**
  ******************************************************************************
  * @file    main.c
  * @author  Weili An
  * @version V1.0
  * @date    Oct 17, 2022
  * @brief   ECE 362 Lab 6 Student template
  ******************************************************************************
*/


#include "stm32f0xx.h"
#include <stdint.h>

void initc();
void initb();
void togglexn(GPIO_TypeDef *port, int n);
void init_exti();
void set_col(int col);
void SysTick_Handler();
void init_systick();
void adjust_priorities();

extern void nano_wait(int);

volatile int current_col = 1;

int main(void) {
    // Uncomment when most things are working
    // autotest();
    
    initb();
    initc();
    init_exti();
    init_systick();
    adjust_priorities();

    // Slowly blinking
    for(;;) {
        togglexn(GPIOC, 9);
        nano_wait(500000000);
    }
}

/**
 * @brief Init GPIO port C
 *        PC0-PC3 as input pins with the pull down resistor enabled
 *        PC4-PC9 as output pins
 * 
 */
void initc() {
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
  GPIOC->MODER &= 0xfff00000;
  GPIOC->MODER |= 0x00055500;
  GPIOC->PUPDR &= 0xffffff00;
  GPIOC->PUPDR |= 0x000000aa;

}

/**
 * @brief Init GPIO port B
 *        PB0, PB2, PB3, PB4 as input pins
 *          enable pull down resistor on PB2 and PB3
 *        PB8-PB11 as output pins
 * 
 */
void initb() {
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
  GPIOB->MODER &= 0xff00fc0c;
  GPIOB->MODER |= 0x00550000;
  GPIOB->PUPDR &= 0xfffffc0c;
  GPIOB->PUPDR |= 0x000000a0;

}

/**
 * @brief Change the ODR value from 0 to 1 or 1 to 0 for a specified 
 *        pin of a port.
 * 
 * @param port : The passed in GPIO Port
 * @param n    : The pin number
 */
 
void togglexn(GPIO_TypeDef *port, int n) {
  if ((port->ODR & (1 << n)) == 1){
    port->BRR |= (1 << n);  
  }
  else {
    port->BSRR |= (1 << n);
  }   
}

//==========================================================
// Write the EXTI interrupt handler for pins 0 and 1 below.
// Copy the name from startup/startup_stm32.s, create a label
// of that name below, declare it to be global, and declare
// it to be a function.
// It acknowledge the pending bit for pin 0, and it should
// call togglexn(GPIOB, 8).

void EXTI0_1_IRQHandler(){
  EXTI->PR |= EXTI_PR_PR0;
  togglexn(GPIOB, 8);
}

//==========================================================
// Write the EXTI interrupt handler for pins 2-3 below.
// It should acknowledge the pending bit for pin2, and it
// should call togglexn(GPIOB, 9).

void EXTI2_3_IRQHandler(){
  EXTI->PR |= EXTI_PR_PR2;
  togglexn(GPIOB, 9);
}

//==========================================================
// Write the EXTI interrupt handler for pins 4-15 below.
// It should acknowledge the pending bit for pin4, and it
// should call togglxn(GPIOB, 10).

void EXTI4_15_IRQHandler(){
  EXTI->PR |= EXTI_PR_PR4;
  togglexn(GPIOB, 10);
}

/**
 * @brief Follow lab manual section 4.4 to initialize EXTI
 *        (1-2) Enable the SYSCFG subsystem, and select Port B for
 *            pins 0, 2, 3, and 4.
 *        (3) Configure the EXTI_RTSR register so that an EXTI
 *            interrupt is generated on the rising edge of
 *            pins 0, 2, 3, and 4.
 *        (4) Configure the EXTI_IMR register so that the EXTI
 *            interrupts are unmasked for pins 2, 3, and 4.
 *        (5) Enable the three interupts for EXTI pins 0-1, 2-3 and
 *            4-15. Don't enable any other interrupts.
 */
void init_exti() {
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
  SYSCFG->EXTICR[0] |= (SYSCFG_EXTICR1_EXTI0_PB|SYSCFG_EXTICR1_EXTI2_PB|SYSCFG_EXTICR1_EXTI3_PB);
  SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI4_PB;
  EXTI -> RTSR |= (EXTI_RTSR_TR0|EXTI_RTSR_TR2|EXTI_RTSR_TR3|EXTI_RTSR_TR4);
  EXTI-> IMR |= (EXTI_IMR_MR0|EXTI_IMR_MR2|EXTI_IMR_MR3|EXTI_IMR_MR4); 
  NVIC_EnableIRQ(EXTI0_1_IRQn);
  NVIC_EnableIRQ(EXTI2_3_IRQn);
  NVIC_EnableIRQ(EXTI4_15_IRQn);
}

/**
 * @brief For the keypad pins, 
 *        Set the specified column level to logic "high.
 *        Set the other three three columns to logic "low".
 * 
 * @param col 
 */
void set_col(int col) {
  GPIOC->ODR &= 0xffffff0f;
  GPIOC->ODR |= 0x00000080 >> (col - 1)
}

/**
 * @brief The ISR for the SysTick interrupt.
 * 
 */
void SysTick_Handler() {
  uint32_t row_value = GPIOC -> IDR & 0xffffffff;
  if (0x16 >> current_col == row_value){
    togglexn(GPIOB,7 + current_col);
  }
  current_col = current_col + 1;
  current_col = (current_col > 4) ? 1 : current_col;
  set_col(current_col);
}


/**
 * @brief Enable the SysTick interrupt to occur every 1/16 seconds.
 * 
 */
void init_systick() {
  SysTick->LOAD = 375000 - 1;
  SysTick->CTRL |= (SysTick_CTRL_ENABLE_Msk|SysTick_CTRL_TICKINT_Msk); 

}

/**
 * @brief Bonus question
 *        Set the priority for EXTI pins 2-3 interrupt to 192.
 *        Set the priority for EXTI pins 4-15 interrupt to 128.
 *        Do not adjust the priority for any other interrupts.
 * 
 */
void adjust_priorities() {
    NVIC_SetPriority(EXTI2_3_IRQn, 192 >> 6);
    NVIC_SetPriority(EXTI4_15_IRQn, 128 >> 6);
}
