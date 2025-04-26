// Systick.c
// Line Following Robot Starter program 
// Created by TYLOR FRANCA 
// February 2025
// CECS 497 SPRING 2025
// California State University, Long Beach
// Modified by Min He and Mark Joseph Austria
// 4/25/2025


#include "tm4c123gh6pm.h"
#include "Systick.h"
#include <stdint.h>

#define ONE_MILLI_S 16000	  // SysTick timer reload value for one millisecond, assume 16MHz system clock.
#define ONE_MICRO_S 16 // SysTick timer reload value for one microsecond, assume 16MHz system clock.

void SysTick_Init(void){
	NVIC_ST_CTRL_R = 0;           			// disable SysTick during setup
	NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0x1FFFFFFF)|0x40000000; // bit 31-29 for SysTick, set priority to 2
	NVIC_ST_CTRL_R = NVIC_ST_CTRL_CLK_SRC + NVIC_ST_CTRL_INTEN;  // enable with core clock (16Mhz) and interrupts, start systick timer
	
}

void DelayMs(void){	
	//set Reload to ONE_MILLI_S
	NVIC_ST_RELOAD_R = ONE_MILLI_S - 1;
  	// Set Current to 0                                       
	NVIC_ST_CURRENT_R = 0;
	// enable SysTick timer
	NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE;
	// wait for COUNT bit in control register to be raised.
	while ((NVIC_ST_CTRL_R & NVIC_ST_CTRL_COUNT) == 0 );
  	// disable SysTick timer
	NVIC_ST_CTRL_R &= ~NVIC_ST_CTRL_ENABLE;
}

// Time delay using busy wait.
// This function will generate multiple of millisecond.
// Input: 32-bit interger for multiple of ms
void Wait_N_MS(uint32_t delay){	
	while(delay){
		DelayMs();
		delay--;
	}
}

void DelayUs(void){
	//set Reload to ONE_MICRO_S
	NVIC_ST_RELOAD_R = ONE_MILLI_S - 1;
  	// Set Current to 0                                       
  	NVIC_ST_CURRENT_R = 0;
	// enable SysTick timer
	NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE;
	// wait for COUNT bit in control register to be raised.
	while ((NVIC_ST_CTRL_R & NVIC_ST_CTRL_COUNT) == 0 );
  	// disable SysTick timer
	NVIC_ST_CTRL_R &= ~NVIC_ST_CTRL_ENABLE;
}

// Time delay using busy wait.
// This function will generate multiple of microsecond.
// Input: 32-bit interger for multiple of us
void Wait_N_US(uint32_t delay){	
	while(delay){
		DelayUs();
		delay--;
	}
}
