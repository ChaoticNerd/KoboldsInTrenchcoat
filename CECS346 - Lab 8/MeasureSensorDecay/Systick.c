// Systick.c
// Sensor Decay Test Code 
// Created by TYLOR FRANCA 
// February 2025
// CECS 497 SPRING 2025
// California State University, Long Beach
// Modified by Min He


#include "tm4c123gh6pm.h"
#include "Systick.h"
#include <stdint.h>

#define ONE_MILLI_S	16				// SysTick timer reload value for one millisecond, assume 16MHz system clock.
#define ONE_MICRO_S 16000     // SysTick timer reload value for one microsecond, assume 16MHz system clock.

// in short: two systick options, ms or us
// two busy waits too

void SysTick_Init(void){
  NVIC_ST_CTRL_R = 0;                   // disable SysTick during setup
  NVIC_ST_CTRL_R |= NVIC_ST_CTRL_CLK_SRC; // set SysTick timer to use core clock: 50MHz
}

void DelayMs(void){	
	NVIC_ST_RELOAD_R = ONE_MILLI_S-1;
  NVIC_ST_CURRENT_R = 0;                // any write to current clears it                                        
  NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE; // enable SysTick timer
	
	// wait for COUNT bit in control register to be raised.
	while ((NVIC_ST_CTRL_R&NVIC_ST_CTRL_COUNT)==0) {} 
  NVIC_ST_CTRL_R &= ~NVIC_ST_CTRL_ENABLE; // disable SysTick timer
}

// Time delay using busy wait.
// This function will generate multiple of millisecond.
// Input: 32-bit interger for multiple of ms
void Wait_N_MS(uint32_t delay){	
	uint32_t i;
	
	for (i=0;i<delay;i++){
		DelayMs();
	}
}

void DelayUs(void){	
	NVIC_ST_RELOAD_R = ONE_MICRO_S-1;
  NVIC_ST_CURRENT_R = 0;                // any write to current clears it                                        
  NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE; // enable SysTick timer
	
	// wait for COUNT bit in control register to be raised.
	while ((NVIC_ST_CTRL_R&NVIC_ST_CTRL_COUNT)==0) {} 
  NVIC_ST_CTRL_R &= ~NVIC_ST_CTRL_ENABLE; // disable SysTick timer
}

// Time delay using busy wait.
// This function will generate multiple of microsecond.
// Input: 32-bit interger for multiple of us
void Wait_N_US(uint32_t delay){	
	uint32_t i;
	// this just fucking calls the systick bruh
	// literally loops 1 microsecond * delay
	// so sensor calls for wait 10 us
	for (i=0;i<delay;i++){
		DelayUs();
	}
}
