// SysTickTestMain.c
// Runs on LM4F120/TM4C123
// Test the SysTick functions by activating the PLL, initializing the
// SysTick timer, and flashing an LED at a constant rate.
// Daniel Valvano
// September 12, 2013
// Modified by Min He on 3/7/2024

/* This example accompanies the books
   "Embedded Systems: Introduction to ARM Cortex M Microcontrollers",
   ISBN: 978-1469998749, Jonathan Valvano, copyright (c) 2013
   Volume 1, Program 4.7

   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2013
   Program 2.11, Section 2.6

 Copyright 2013 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

#include "tm4c123gh6pm.h"
#include "SysTick.h"
#include "PLL.h"

#define GREEN_LED       (*((volatile unsigned long *)0x40025020))
#define GREEN_LED_MASK    0x08  // bit position for onboard green LED

void PORTF_Init(void);

int main(void){
	PORTF_Init();							// PF2(Blue LED) is an output for debugging
  PLL_Init();               // set system clock to 50 MHz
  SysTick_Init();           // initialize SysTick timer
  while(1){
    GREEN_LED ^= GREEN_LED_MASK; // toggle PF2: Blue LED
    SysTick_Wait10ms(50);    // approximately 10*10 ms = 0.1s
  }
}

void PORTF_Init(void)
{
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5; // activate port F
	while ((SYSCTL_RCGCGPIO_R&SYSCTL_RCGCGPIO_R5)!=SYSCTL_RCGCGPIO_R5){};
	
	GPIO_PORTF_DIR_R |= GREEN_LED_MASK; // make PF2 out (built-in blue LED)
  GPIO_PORTF_AFSEL_R &= ~GREEN_LED_MASK;// disable alt funct on PF2
  GPIO_PORTF_DEN_R |= GREEN_LED_MASK; // enable digital I/O on PF2                            
  GPIO_PORTF_PCTL_R &= ~0x00000F00; // configure PF2 as GPIO
  GPIO_PORTF_AMSEL_R |= GREEN_LED_MASK;   // disable analog functionality on PF  
}
