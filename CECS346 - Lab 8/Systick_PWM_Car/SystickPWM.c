/////////////////////////////////////////////////////////////////////////////
// SystickPWM.c
// Example project for using Systick timer to generate PWM signals to control 
// the movement of a DC motor Car.
// Description: 
// Hardware connections: This program works for Romi chassis 
// PB67 for motor PWM signals: PB6 - Left DC Motor, PB7 - Right DC Motor
// PB5432 for motor directions: PB54 - left DC Motor, PB32 - right DC Motor
// By Min He, 4/20/2025
/////////////////////////////////////////////////////////////////////////////

#include "tm4c123gh6pm.h"
#include <stdint.h>
#include "SystickPWM.h"

// bit address definitions for port data registers
// PB5432 are the four direction pins:PB5:L:SLP,PB4:L:DIR, PB3:R:SLP,PB2:R:DIR
#define DIRECTION (*((volatile uint32_t *)0x400050F0)) 
#define MOTORS (*((volatile uint32_t *)0x40005300)) 

// Constant definitions for direction pins based on the following hardware connections:
// PB5432 are used for direction control.
// PB54 is connected to the left motor, PB32 is connected to the right motor.
// PB5:L:SLP,PB4:L:DIR, PB3:R:SLP,PB2:R:DIR
// SLP|DIR: 11: forward
// SLP|DIR: 10: backward
//#define FORWARD 0x3C    // both motors forward
//#define BACKWARD 0x28   // both motors backward
#define LEFTPIVOT 0x2C  // left motor backward, right motor forward
#define RIGHTPIVOT 0x38 // left motor forward, right motor backward 

// bits 6,7 are PWM outputs connected to DC Motor: bit 6->left motor, bit 7->right motor
#define PWM_LEFT  0x80
#define PWM_RIGHT 0x40

// Global variables: 
// H: number of clocks cycles for duty cycle
// L: number of clock cycles for non-duty cycle
volatile uint32_t H,L;


void SysTick_Init(void){
  NVIC_ST_CTRL_R = 0;           			// disable SysTick during setup
  NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0x1FFFFFFF)|0x40000000; // bit 31-29 for SysTick, set priority to 2
  NVIC_ST_CTRL_R = NVIC_ST_CTRL_CLK_SRC + NVIC_ST_CTRL_INTEN;  // enable with core clock and interrupts, start systick timer
}


// This function initialize PB7-PB2 to output PWM signals and 
// direction signals for Two DC Motors:
// PB67 for motor PWM signals: PB6 - Left DC Motor, PB7 - Right DC Motor
// PB5432 for motor directions: PB54 - left DC Motor, PB32 - right DC Motor
void Motor_Init(uint32_t speed){
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOB;     	// activate F clock
	L = speed;
	H = PERIOD - H;
	
  GPIO_PORTB_AMSEL_R &= ~0xFC;  			// disable analog functionality 
  GPIO_PORTB_PCTL_R &= ~0xFFFFFF00; 	// configure as GPIO
  GPIO_PORTB_DIR_R |= 0xFC;     			// set as output
  GPIO_PORTB_AFSEL_R &= ~0xFC;  			// disable alt funct 
  GPIO_PORTB_DEN_R |= 0xFC;     			// enable digital I/O 
	SysTick_Init();                     // Systick timer is used to control both PWM outputs at PB67
}

// This function will start motor in the direction specified by dir.
void Motor_Start(void)
{
  NVIC_ST_RELOAD_R = H-1;       // reload value in number of clock cycles
  NVIC_ST_CURRENT_R = 0;        // any write to current clears it
	NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE;
}

// This function will stop motor movement.
void Motor_Stop(void)
{
	NVIC_ST_CTRL_R &= ~NVIC_ST_CTRL_ENABLE;
	GPIO_PORTB_DATA_R &= ~(PWM_RIGHT|PWM_LEFT); // output zero for PWM
}

// SysTick ISR:
// 1. Implement timing control for duty cycle and non-duty cycle
// 2. Output a waveform based on current duty cycle
void SysTick_Handler(void){
	
	// add LED
  if(MOTORS&pwm){   // previous cycle is duty cycle
    NVIC_ST_RELOAD_R = L-1;     // switch to non-duty cycle
  } else{ // previous cycle is non-duty cycle
    NVIC_ST_RELOAD_R = H-1;     // switch to duty cycle
  }
	
	MOTORS ^= pwm; // inverse output for PWM
}
