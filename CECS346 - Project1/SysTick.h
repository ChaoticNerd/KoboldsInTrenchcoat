// Lab4SysTick.h
// Runs on TM4C123
// Provide functions that initialize the SysTick module and 
// function to generate a multiple of 10 milliseconds using busy wait.  
// Original author: Daniel Valvano,Jonathan W. Valvano September 11, 2013
// Modified by Min He on 9/24/2020, 2/22/2025.

// checks if __LAB4SYSTICK_H__ has been defined earlier in this file or in another file
// if it isn't, continue
// wait then the c??
// supposed to prevent running this file twice and redeclaring variables and making errors
#ifndef __LAB4SYSTICK_H__
// declare __LAB4SYSTICK_H__ so that the code below runs
#define __LAB4SYSTICK_H__

#include <stdint.h>

// Initialize SysTick with busy wait running at bus clock. 
// Do not enable SysTick timer in this function
#define NVIC_ST_CTRL_R          (*((volatile uint32_t *)0xE000E010))
#define NVIC_ST_RELOAD_R        (*((volatile uint32_t *)0xE000E014))
#define NVIC_ST_CURRENT_R       (*((volatile uint32_t *)0xE000E018))
#define NVIC_ST_CTRL_COUNT      (0x00010000)  // Count flag (turns on counter, becomes 0 when timer ends)
#define NVIC_ST_CTRL_CLK_SRC    (0x00000004)  // Clock Source (1 for internal at bit 2)
#define NVIC_ST_CTRL_ENABLE     (0x00000001)  // Counter mode (enable counter)
#define HALF_SEC_RELOAD 				(8000000)  // reload value for generating 500ms time interval for 16 MHz system clock.

// Initialize SysTick with busy wait running at bus clock. 
// Do not enable SysTick timer in this function
void SysTick_Init(void){
	NVIC_ST_CTRL_R &= ~NVIC_ST_CTRL_ENABLE; // disable SysTick when setting up
	NVIC_ST_CTRL_R |= NVIC_ST_CTRL_CLK_SRC; // source clock from internal system
}

// Time delay using busy wait assumes 16 MHz system clock.
// This function generate a time delay of 0.5s.
// Implementaion requirement: enable the timer before generating delay and 
// disable it after 0.5s is generated.
// NOTE: Use SysTick_Wait10ms as an example to learn how to generate 0.5s delay

// function is for resetting and starting clock again
void SysTick_Wait_HalfSec(void){
	NVIC_ST_RELOAD_R = HALF_SEC_RELOAD - 1; // countdown from this number to 0
	NVIC_ST_CURRENT_R = 0; // clear countdown counter
	NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE; // enable SysTick timer
	while ((NVIC_ST_CTRL_R & NVIC_ST_CTRL_COUNT) == 0); // busy wait timer
	NVIC_ST_CTRL_R &= ~NVIC_ST_CTRL_ENABLE; // disable SysTick again to set up
}

// This function will generate multiple 0.5s delay
// Parameter: number of half seconds
// n_half_s: specify how many 0.5s will be generated
// maximum delay can be generated: 2^32*0.5s
void Wait_N_Half_Sec( uint32_t n_half_s) {
	while (n_half_s) {
		SysTick_Wait_HalfSec();
		n_half_s--;
	}
}

#endif // __LAB4SYSTICK_H__