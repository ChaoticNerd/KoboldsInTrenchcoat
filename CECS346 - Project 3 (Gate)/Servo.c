//Servo.c
// Example program to test servo motor.
// This file provide driver functions to control a servo motor.
// By Min He
// 4/17/2025
#include "tm4c123gh6pm.h"
#include <stdint.h>
#include "Servo.h"

// priority 4 systick
#define SERVO (*((volatile uint32_t *)0x40005100))  // use PB6 for Servo

#define SERVO_BIT_MASK 	0x40								//
#define SERVO_PCTL_MASK 0x0F000000					//

#define PRI3_TOP3_BITS_RESET	0x1FFFFFFFF 	// used to clear all priority bits of systick
#define PRI3_TOP3_BITS_SET		0x800000000		// used to set systick priority as 4
#define Duty_Cycle_toggle 		1 						// Duty cycle to toggle value
#define Systick_reset 				0							// Valur to set count to 0
// high(duty cycle) and low(non-duty cycle) reload values
static uint32_t H;
static uint32_t L;			

void SysTick_Init(void){
  NVIC_ST_CTRL_R &= ~NVIC_ST_CTRL_ENABLE;    																			// disable SysTick during setup
  NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&PRI3_TOP3_BITS_RESET)|PRI3_TOP3_BITS_SET; 		// bit 31-29 for SysTick, set priority to 4
	NVIC_ST_CTRL_R |= NVIC_ST_CTRL_CLK_SRC | NVIC_ST_CTRL_INTEN;										// enable SysTick clock source, interrupt
}

void Servo_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;																//Activate GPIOB Clock
	while((SYSCTL_RCGCGPIO_R & SYSCTL_RCGCGPIO_R1) != SYSCTL_RCGCGPIO_R1);	
	
	GPIO_PORTB_AFSEL_R 	&= ~SERVO_BIT_MASK;							// Disable Alternate Function
	GPIO_PORTB_PCTL_R 	&= ~SERVO_PCTL_MASK;				    // set to GPIO
	GPIO_PORTB_DIR_R 		|= SERVO_BIT_MASK;							// set to output pin
	GPIO_PORTB_AMSEL_R 	&= ~SERVO_BIT_MASK;							// Disable Analog Function
	GPIO_PORTB_DEN_R 		|= SERVO_BIT_MASK;							// Enable Digital I/O
}

void Drive_Servo(uint32_t angle){
	H = angle;																// Defines reload High
	L = SERVO_PERIOD - H;											// Defines reload Low
	SERVO |= SERVO_BIT_MASK;									// Start Servo with high
	NVIC_ST_RELOAD_R = H;											// Sets  reload to High
	NVIC_ST_CURRENT_R = Systick_reset ;				// clear SysTick Count
	NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE;		// enable SysTick
}

void SysTick_Handler(void){	
	NVIC_ST_CTRL_R &= ~NVIC_ST_CTRL_ENABLE;	 // clears enable
	if(SERVO){                               // previous cycle is duty cycle
		NVIC_ST_RELOAD_R = L-Duty_Cycle_toggle;// switch to non-duty cycle
	} else{ 															   // previous cycle is not duty cycle
		NVIC_ST_RELOAD_R = H-Duty_Cycle_toggle;// switch to duty cycle
	}
	SERVO ^= SERVO_BIT_MASK;								 // inverse Servo output
	NVIC_ST_CURRENT_R = Systick_reset ;			 // set SysTick Count to 0
	NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE;	 // enable SysTick
}