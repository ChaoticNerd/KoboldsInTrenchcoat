// SysTick.c
// Runs on TM4C123
// Starter file for CECS346 Project 2
// By Dr. Min He


// oscilloscope or LED connected to PF2 for period measurement

#include "tm4c123gh6pm.h"
#include <stdint.h> // C99 data types

#define HALF_SEC_RELOAD 8000000

// initialize SysTick
void SysTick_Init(void) {	
    NVIC_ST_CTRL_R = 0; // disable SysTick when setting up
	NVIC_ST_CTRL_R |= NVIC_ST_CTRL_CLK_SRC; // sets SysTick to source clock from internal system
}

// 
void SysTick_Start(uint32_t period) {	
	while (period) {
		NVIC_ST_RELOAD_R = HALF_SEC_RELOAD - 1; 							// countdown from this number to 0
		NVIC_ST_CURRENT_R = 0; 																// clear countdown counter
		NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE; 								// enable SysTick timer
		while ((NVIC_ST_CTRL_R & NVIC_ST_CTRL_COUNT) == 0); 	// busy wait timer
		NVIC_ST_CTRL_R &= ~NVIC_ST_CTRL_ENABLE; 							// disable SysTick again to set up
		period--;
	}
}


//
void SysTick_Stop(void) {
    NVIC_ST_CTRL_R &= ~NVIC_ST_CTRL_ENABLE; // disable SysTick again to set up
		NVIC_ST_CURRENT_R = 0; 									// clear countdown counter
}