/////////////////////////////////////////////////////////////////////////////
// SystickPWM.h
// Example project for using Systick timer to generate PWM signals to control 
// the speed of a DC motor Car
// Description: 
// Hardware connections: assume L298N is used for DC motor driver
// PB67 for motor PWM signals: PB6 - Left DC Motor, PB7 - Right DC Motor
// PB5432 for motor directions: PB54 - left DC Motor, PB32 - right DC Motor
// By Min He, 4/20/2023
/////////////////////////////////////////////////////////////////////////////

#include "tm4c123gh6pm.h"
#include <stdint.h>

// Constants
#define PERIOD 			160000              // PWM Period:10ms, value is based on 16MH system clock
#define SLOW				PERIOD*0.3
#define MID		      PERIOD*0.5
#define FAST		    PERIOD*0.9

// constant definitions for different moving directions.
enum DIR {FORWARD_STRAIGHT, BACKWARD_STRAIGHT,FORWARD_LEFT,FORWARD_RIGHT,BACKWARD_LEFT,BACKWARD_RIGHT,PIVOT_CCW, PIVOT_CW};

// This function initialize PB7-PB2 to output PWM signals and 
// direction signals for Two DC Motors:
// PB67 for motor PWM signals: PB6 - Left DC Motor, PB7 - Right DC Motor
// PB5432 for motor directions: PB54 - left DC Motor, PB32 - right DC Motor
void Motor_Init(uint32_t speed);
	
// This function will start motor in the direction specified by dir.
void Motor_Start(uint8_t dir);

void Motor_Clear(void);

// This function will stop motor movement.
void Motor_Stop(void);

void SysTick_Init(void);

void Wait_N_MS(uint32_t delay);

void Wait_N_US(uint32_t delay);
