/**
  ******************************************************************************
  * @file    main.c
  * @author  Weili An
  * @version V1.0
  * @date    Oct 24, 2022
  * @brief   ECE 362 Lab 7 template
  ******************************************************************************
*/


#include "stm32f0xx.h"
#include <stdint.h>

// Global data structure
char* login          = "xyz"; // Replace with your login.
char disp[9]         = "Hello...";
uint8_t col          = 0;
uint8_t mode         = 'A';
uint8_t thrust       = 0;
int16_t fuel         = 800;
int16_t alt          = 4500;
int16_t velo         = 0;

// Make them visible to autotest.o
extern char* login;
// Keymap is in `font.S` to match up what autotester expected
extern char keymap;
extern char disp[9];
extern uint8_t col;
extern uint8_t mode;
extern uint8_t thrust;
extern int16_t fuel;
extern int16_t alt;
extern int16_t velo;

char* keymap_arr = &keymap;

// Font array in assembly file
// as I am too lazy to convert it into C array
extern uint8_t font[];

// The functions we should implement
void enable_ports();
void setup_tim6();
void show_char(int n, char c);
void drive_column(int c);
int read_rows();
char rows_to_key(int rows);
void handle_key(char key);
void setup_tim7();
void write_display();
void update_variables();
void setup_tim14();

// Auotest functions
extern void check_wiring();
extern void autotest();
extern void fill_alpha();

int main(void) {
    // check_wiring();
    autotest();
    // fill_alpha();
    enable_ports();
    setup_tim6();
    setup_tim7();
    setup_tim14();

    for(;;) {
        asm("wfi");
    }
}

/**
 * @brief Enable the ports and configure pins as described
 *        in lab handout
 * 
 */
void enable_ports(){
  RCC->AHBENR |= (RCC_AHBENR_GPIOCEN|RCC_AHBENR_GPIOBEN);

  GPIOB->MODER &= ~(0x3fffff);
  GPIOB->MODER |= (0x155555);

  GPIOC->MODER &= ~(0x3ffff);
  GPIOC->MODER |= 0x15500;
  GPIOC->PUPDR &= ~(0xff);
  GPIOC->PUPDR |= (0xaa);
}


//-------------------------------
// Timer 6 ISR goes here
//-------------------------------
// TODO
void TIM6_DAC_IRQHandler(){
  TIM6->SR &= ~(TIM_SR_UIF); //Acknowledge the interrupt
  //Toggle PC8
  if (GPIOC->ODR & (0x1 << 8)){
    GPIOC->BSRR |= GPIO_BSRR_BR_8;} 
  else {
    GPIOC->BSRR |= GPIO_BSRR_BS_8;}
}

/**
 * @brief Set up timer 6 as described in handout
 * 
 */
void setup_tim6() {
  RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

  TIM6->PSC = 48000 -1;
  TIM6->ARR = 500 - 1;
  TIM6->DIER |= TIM_DIER_UIE;
  TIM6->CR1 |= TIM_CR1_CEN;

  // NVIC_EnableIRQ(TIM6_DAC_IRQn);
  NVIC->ISER[0] = 1 << TIM6_DAC_IRQn;

}

/**
 * @brief Show a character `c` on column `n`
 *        of the segment LED display
 * 
 * @param n 
 * @param c 
 */
void show_char(int n, char c) {
  if ((n >= 0) && (n <= 7)){
    GPIOB->ODR |= (font[c] | n << 8); 
  }
}

/**
 * @brief Drive the column pins of the keypad
 *        First clear the keypad column output
 *        Then drive the column represented by `c`
 * 
 * @param c 
 */
void drive_column(int c) {
  c = 0b11 & c;
  GPIOC->BRR |= 0xf0;
  GPIOC->BSRR |= 1 << (c + 4);
}

/**
 * @brief Read the rows value of the keypad
 * 
 * @return int 
 */
int read_rows() {
  uint32_t rows = (GPIOC->IDR & 0XF);
  return(rows);
}

/**
 * @brief Convert the pressed key to character
 *        Use the rows value and the current `col`
 *        being scanning to compute an offset into
 *        the character map array
 * 
 * @param rows 
 * @return char 
 */
char rows_to_key(int rows) {

}

/**
 * @brief Handle key pressed in the game
 * 
 * @param key 
 */
void handle_key(char key) {

}

//-------------------------------
// Timer 7 ISR goes here
//-------------------------------
// TODO
void TIM7_IRQHandler(){
  //acknowledge the interrupt first
  TIM7->SR &= ~(TIM_SR_UIF);
  
  if (read_rows() != 0) {
      handle_key(rows_to_key(read_rows()));
  }

  show_char(col, disp[col]);
  col += 1;
  col = col > 7 ? 0 : col; 
  drive_column(col);
}

/**
 * @brief Setup timer 7 as described in lab handout
 * 
 */
void setup_tim7() {
  RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
  TIM7->PSC = 4800 - 1;
  TIM7->ARR = 10 - 1;
  TIM7->DIER |= TIM_DIER_UIE;
  TIM7->CR1 |= TIM_CR1_CEN;

  // NVIC_EnableIRQ(TIM7_IRQn);
  NVIC->ISER[0] = 1 << TIM7_IRQn;

}

/**
 * @brief Write the display based on game's mode
 * 
 */
void write_display() {

}

/**
 * @brief Game logic
 * 
 */
void update_variables() {

}

//-------------------------------
// Timer 14 ISR goes here
//-------------------------------
// TODO
void TIM14_IRQHandler(){
  TIM14->SR &= ~(TIM_SR_UIF);
  update_variables();
  write_display();
}

/**
 * @brief Setup timer 14 as described in lab
 *        handout
 * 
 */
void setup_tim14() {
  RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
  TIM14->PSC = 24000 - 1;
  TIM14->ARR = 1000 - 1;
  TIM14->DIER |= TIM_DIER_UIE; 
  TIM14->CR1 |= TIM_CR1_CEN; 
  NVIC_EnableIRQ(TIM14_IRQn);
}
