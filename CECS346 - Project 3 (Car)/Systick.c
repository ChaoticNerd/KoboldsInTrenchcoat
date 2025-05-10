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
#define RELOAD_SHIFT	1
#define CURRENT_CLEAR	0
#define WAIR_FOR_RAISE	0

void SysTick_Init(void){
	NVIC_ST_CTRL_R &= ~NVIC_ST_CTRL_ENABLE;   // disable SysTick during setup
	NVIC_ST_CTRL_R |= NVIC_ST_CTRL_CLK_SRC;  // enable systick
}

void DelayMs(void){	
	//set Reload to ONE_MILLI_S
	NVIC_ST_RELOAD_R = ONE_MILLI_S - RELOAD_SHIFT;
  	// Set Current to 0                                       
	NVIC_ST_CURRENT_R = CURRENT_CLEAR;
	// enable SysTick timer
	NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE;
	// wait for COUNT bit in control register to be raised.
	while ((NVIC_ST_CTRL_R & NVIC_ST_CTRL_COUNT) == WAIR_FOR_RAISE );
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
	NVIC_ST_RELOAD_R = ONE_MICRO_S - RELOAD_SHIFT;
  	// Set Current to 0                                       
  	NVIC_ST_CURRENT_R = CURRENT_CLEAR;
	// enable SysTick timer
	NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE;
	// wait for COUNT bit in control register to be raised.
	while ((NVIC_ST_CTRL_R & NVIC_ST_CTRL_COUNT) == WAIR_FOR_RAISE );
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
