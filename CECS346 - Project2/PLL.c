// PLL.c
// Runs on TM4C123
// Starter file for CECS346 Project 2
// By Dr. Min He
// Edited by Natasha Kho, Justin Narciso, Hanna Estrada, William Grefaldeo
 
#include "PLL.h"
#include "tm4c123gh6pm.h"

// The #define statement SYSDIV2 in PLL.h
// initializes the PLL to the desired frequency.

// bus frequency is 400MHz/(SYSDIV2+1) = 400MHz/(7+1) = 50 MHz
// see the table at the end of this file

#define CLR_SYSCLK_DIV  0x1FC00000  // used to clear system clock divider field

// configure the system to get its clock from the PLL
void PLL_Init(void){
    // Set system to use Run-Mode Clock Configuration2 and non-int System Clock Divisor
    SYSCTL_RCC2_R |= SYSCTL_RCC2_USERCC2; // overrides RCC

    // Bypass PLL while initializing
    SYSCTL_RCC2_R |= SYSCTL_RCC2_BYPASS2; // System clock is set to OSC (400MHz) and divided by SYSDIV2

     // 2) select the crystal value and oscillator source
    SYSCTL_RCC_R &= ~SYSCTL_RCC_XTAL_M;   // clear XTAL field: 0x7C0=> 011111000000
    SYSCTL_RCC_R += SYSCTL_RCC_XTAL_16MHZ; // configure for 16 MHz crystal: 10101=>0x15
    SYSCTL_RCC2_R &= ~SYSCTL_RCC2_OSCSRC2_M; // clear oscillator source field
    SYSCTL_RCC2_R += SYSCTL_RCC2_OSCSRC2_MO; // configure for main oscillator source

	// 3) activate PLL by clearing PWRDN
    SYSCTL_RCC2_R &= ~SYSCTL_RCC2_PWRDN2; // PLL acts normally

    // TODO: PUT MORE COMMENTS
    // 4) set the desired system divider and the system divider least significant bit
    SYSCTL_RCC2_R |= SYSCTL_RCC2_DIV400;         // use 400 MHz PLL (max)
    // SLIDE 8, LECTURE 5 FOR GRAPHS
    SYSCTL_RCC2_R = (SYSCTL_RCC2_R&~CLR_SYSCLK_DIV) // clear system clock divider field: setting up bit 22 to 28
	+ (SYSDIV2<<22);      // configure for 16 MHz clock: assign value 24 to 7 bits(0x0001111(hex), 24(dec)), then move it to the right field: bit 22 to 28
  
    // 5) wait for the PLL to lock by polling PLLLRIS
    while((SYSCTL_RIS_R&SYSCTL_RIS_PLLLRIS)==0){};
  
    // 6) enable use of PLL by clearing BYPASS
    SYSCTL_RCC2_R &= ~SYSCTL_RCC2_BYPASS2;  // System clock is PLL output clock divided by SYSDIV2
}
