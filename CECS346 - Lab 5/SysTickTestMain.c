// SysTickTestMain.c
// Runs on TM4C123
// Provide functions that initialize the SysTick module and 
// function to generate a multiple of 10 milliseconds using busy wait.  
// Original author: Daniel Valvano,Jonathan W. Valvano September 11, 2013
// Modified by Min He on 9/24/2020, 2/22/2025.

// PF2 connects to blue LED. It is an output for debugging.

#include "SysTick.h"
#include <stdint.h>

#define LED                     (*((volatile uint32_t *)0x40025010)) // bit address for GPIO_PORTF_DATA_R bit 2
#define GPIO_PORTF_DIR_R        (*((volatile uint32_t *)0x40025400))
#define GPIO_PORTF_AFSEL_R      (*((volatile uint32_t *)0x40025420))
#define GPIO_PORTF_DEN_R        (*((volatile uint32_t *)0x4002551C))
#define GPIO_PORTF_AMSEL_R      (*((volatile uint32_t *)0x40025528))
#define GPIO_PORTF_PCTL_R       (*((volatile uint32_t *)0x4002552C))
#define SYSCTL_RCGC2_R          (*((volatile uint32_t *)0x400FE108))
#define SYSCTL_RCGCGPIO_R       (*((volatile uint32_t *)0x400FE608))
#define SYSCTL_RCGC2_GPIOF      0x00000020  // port F Clock Gating Control
#define SYSCTL_RCGCGPIO_GPIOF   0x00000020  // port F Clock Gating Control
#define BLUE_LED 					      0x04        // bit position for onboard blue LED.
#define GPIO_PCTL_BLUE_LED			0x00000F00  // bit positions for GPIO pin 2 blue LED.

void PORTF_Init(void);

int main(void){
	PORTF_Init();
  SysTick_Init();           // initialize SysTick timer
  while(1){
    LED = LED^BLUE_LED; 				// toggle PF2:00000100: xor 1->inverse
//    SysTick_Wait10ms(1);    // approximately 10 ms
//    SysTick_Wait10ms(50);    // approximately 50*10 ms=500ms=0.5s
    SysTick_Wait10ms(200);    // approximately 200*10 ms=2000ms=2s		
  }
}

void PORTF_Init(void){
  // Legacy register for activating port F clock
//  SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF; // activate port F
//	while ((SYSCTL_RCGC2_R&SYSCTL_RCGC2_GPIOF)!=SYSCTL_RCGC2_GPIOF){}

  // Currently used register for activating port F clock
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_GPIOF; // activate port F
	while ((SYSCTL_RCGCGPIO_R&SYSCTL_RCGCGPIO_GPIOF)!=SYSCTL_RCGCGPIO_GPIOF){}

		
  GPIO_PORTF_DIR_R |= BLUE_LED; // make PF2 out (built-in blue LED)
  GPIO_PORTF_AFSEL_R &= ~BLUE_LED;// disable alt funct on PF2
  GPIO_PORTF_DEN_R |= BLUE_LED; // enable digital I/O on PF2                            
	GPIO_PORTF_PCTL_R &= ~GPIO_PCTL_BLUE_LED; // configure PF2:blue LED as GPIO
  GPIO_PORTF_AMSEL_R &= ~BLUE_LED;   // disable analog functionality on PF  
}
