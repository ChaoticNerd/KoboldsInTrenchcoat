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

#define SERVO_BIT_MASK 	0x40
#define SERVO_PCTL_MASK 0x0F000000

// high(duty cycle) and low(non-duty cycle) reload values
static uint32_t H;
static uint32_t L;			

void SysTick_Init(void){
  NVIC_ST_CTRL_R &= ~NVIC_ST_CTRL_ENABLE;    										// disable SysTick during setup
  NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0x1FFFFFFF)|0x80000000; 		// bit 31-29 for SysTick, set priority to 4
	NVIC_ST_CTRL_R |= NVIC_ST_CTRL_CLK_SRC | NVIC_ST_CTRL_INTEN;	// enable SysTick clock source, interrupt
}

void Servo_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;				//Activate GPIOB Clock
	while((SYSCTL_RCGCGPIO_R & SYSCTL_RCGCGPIO_R1) != SYSCTL_RCGCGPIO_R1);
	
	GPIO_PORTB_AFSEL_R 	&= ~SERVO_BIT_MASK;							//Enable Alternate Function
	GPIO_PORTB_PCTL_R 	&= ~SERVO_PCTL_MASK;				    //set to GPIO
	GPIO_PORTB_DIR_R 		|= SERVO_BIT_MASK;							//set to output pin
	GPIO_PORTB_AMSEL_R 	&= ~SERVO_BIT_MASK;							//Disable Analog Function
	GPIO_PORTB_DEN_R 		|= SERVO_BIT_MASK;							//Enable Digital I/O
	
}

void Drive_Servo(uint32_t angle){
	H = angle;
	L = SERVO_PERIOD - H;
	SERVO |= SERVO_BIT_MASK;									// Start Servo with high
	NVIC_ST_RELOAD_R = H;
	NVIC_ST_CURRENT_R = 0;										// clear SysTick Count
	NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE;		// enable SysTick
}

void Stop_Servo(void) {
	NVIC_ST_CTRL_R &= ~NVIC_ST_CTRL_ENABLE;		// disable SysTick
	SERVO &= ~SERVO_BIT_MASK;									// output low to Servo 
}

void SysTick_Handler(void){	
	NVIC_ST_CTRL_R &= ~NVIC_ST_CTRL_ENABLE;
	if(SERVO){                               // previous cycle is duty cycle
		NVIC_ST_RELOAD_R = L-1;     				   // switch to non-duty cycle
	} else{ 															   // previous cycle is not duty cycle
		NVIC_ST_RELOAD_R = H-1;     				   // switch to duty cycle
	}
	SERVO ^= SERVO_BIT_MASK;								 // inverse Servo output
	NVIC_ST_CURRENT_R = 0;									 // set SysTick Count to 0
	NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE;	 // enable SysTick
}