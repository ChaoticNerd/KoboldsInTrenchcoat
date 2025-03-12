// SysTick.c
// Runs on TM4C123
// Provide functions that initialize the SysTick module and 
// function to generate a multiple of 10 milliseconds using busy wait.  
// Original author: Daniel Valvano,Jonathan W. Valvano September 11, 2013
// Modified by Min He on 9/24/2020, 2/22/2025.

#include <stdint.h>

#define NVIC_ST_CTRL_R          (*((volatile uint32_t *)0xE000E010))
#define NVIC_ST_RELOAD_R        (*((volatile uint32_t *)0xE000E014))
#define NVIC_ST_CURRENT_R       (*((volatile uint32_t *)0xE000E018))
#define NVIC_ST_CTRL_COUNT      0x00010000  // Count flag
#define NVIC_ST_CTRL_CLK_SRC    0x00000004  // Clock Source
#define NVIC_ST_CTRL_ENABLE     0x00000001  // Counter mode
#define TEN_MINI_SEC 					  160000  // reload value for generating 10ms time interval for 16 MHz system clock.

// Initialize SysTick with busy wait running at bus clock.
void SysTick_Init(void){
  NVIC_ST_CTRL_R = 0;// Clear all settings
  NVIC_ST_CTRL_R |= NVIC_ST_CTRL_CLK_SRC; // setup clock source to be system clock
}

// Time delay using busy wait.
// The delay parameter is in units of the core clock. (units of 62.5 nsec for 16 MHz clock)
void SysTick_Wait(uint32_t delay){
  NVIC_ST_RELOAD_R = delay - 1; // number of counts to wait  
	NVIC_ST_CURRENT_R = 0; // any value written to CURRENT clears  
  NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE;			// enable SysTick timer
	while ((NVIC_ST_CTRL_R & NVIC_ST_CTRL_COUNT) == 0) { } // wait for COUNT
  NVIC_ST_CTRL_R &= ~NVIC_ST_CTRL_ENABLE;			// disable SysTick timer
}

// Time delay using busy wait.
// This assumes 16 MHz system clock.
// This function generate a time delay that is multiple of 10ms
// Parameter:
// n_ms: specify how many 10ms will be generated.
// maximum delay can be generated: 2^32*10ms
void SysTick_Wait10ms(uint32_t n_10ms){
  uint32_t i;
  for(i=0; i<n_10ms; i++){
    SysTick_Wait(TEN_MINI_SEC);  // wait 10ms (based on 16 MHz system clock)
  }
}
