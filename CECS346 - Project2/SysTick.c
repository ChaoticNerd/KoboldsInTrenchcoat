// SysTick.c
// Runs on TM4C123
// Starter file for CECS346 Project 2
// By Dr. Min He
// Edited by Natasha Kho, Justin Narciso


// oscilloscope or LED connected to PF2 for period measurement

#include "tm4c123gh6pm.h"
#include <stdint.h> // C99 data types


#define EN_SYSTICK_CC		NVIC_ST_CTRL_CLK_SRC + NVIC_ST_CTRL_INTEN + NVIC_ST_CTRL_INTEN
#define PRI3_TOP3_BITS_RESET	0x1FFFFFFFF
#define PRI3_TOP3_BITS_SET	0x400000000

// initialize SysTick
void SysTick_Init(void) {	
    NVIC_ST_CTRL_R = 0; // disable SysTick when setting up
	
											// CLEARING BITS HERE										||| ASSIGNING PRIORITY HERE
	  NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R & PRI3_TOP3_BITS_RESET) | PRI3_TOP3_BITS_SET ;		// SLIDE 27 LECTURE 6!!!
		NVIC_ST_CTRL_R = EN_SYSTICK_CC; // sets SysTick to source clock from internal system
}

// 
void SysTick_Start(uint32_t period) {	
	NVIC_ST_RELOAD_R = period - 1; 												// countdown from this number to 0
	NVIC_ST_CURRENT_R = 0; 																// clear countdown counter
	NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE; 								// enable SysTick timer
	while ((NVIC_ST_CTRL_R & NVIC_ST_CTRL_COUNT) == 0); 	// busy wait timer
	NVIC_ST_CTRL_R &= ~NVIC_ST_CTRL_ENABLE; 							// disable SysTick again to set up
}


//
void SysTick_Stop(void) {
    NVIC_ST_CTRL_R &= ~NVIC_ST_CTRL_ENABLE; // disable SysTick again to set up
		NVIC_ST_CURRENT_R = 0; 									// clear countdown counter
}