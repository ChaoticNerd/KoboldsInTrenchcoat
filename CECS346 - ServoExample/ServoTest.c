// ServoTest.c
// Example program to test servo motor.
// This program repeated move the servo in the following sequence:
// center->CW 45 degree->center->CCW 45 degree->center->CW 90 degree->center->CCW 90 degree
// By Min He
// 4/17/2025

#include "tm4c123gh6pm.h"
#include "Servo.h"
#include <stdint.h>

// Function Prototypes
void Delay_N_HalfSec(uint8_t n_500ms);

extern void EnableInterrupts(void);
extern void WaitForInterrupt(void);
extern void DisableInterrupts(void);

// Constants definitions
#define HALF_SEC (727240 * 200 / 91)

// Subroutines Section
// MAIN: Mandatory for a C Program to be executable
int main(void){    
	DisableInterrupts();
  Servo_Init();
	SysTick_Init();
	EnableInterrupts();
	
	while(1) {
		// Test the following servo movements:
		// center->CW 45 degree->center->CCW 45 degree->center->CW 90 degree->center->CCW 90 degree
		Drive_Servo(SERVO_CENTER);		// turn to center position
		Delay_N_HalfSec(1);
		Drive_Servo(SERVO_CW_45);		  // turn clockwise 45 degrees
		Delay_N_HalfSec(1);
		
		Drive_Servo(SERVO_CENTER);		// turn to center position
		Delay_N_HalfSec(1);
		Drive_Servo(SERVO_CCW_45);		// turn counterclockwise 45 degrees
		Delay_N_HalfSec(1);
		
		Drive_Servo(SERVO_CENTER);		// turn to center position
		Delay_N_HalfSec(1);
		Drive_Servo(SERVO_CW_90);		  // turn clockwise 90 degrees
		Delay_N_HalfSec(1);
		
		Drive_Servo(SERVO_CENTER);		// turn to center position
		Delay_N_HalfSec(1);
		Drive_Servo(SERVO_CCW_90);		// turn counterclockwise 90 degrees
		Delay_N_HalfSec(1);		
	}
}

// Subroutine to wait multiple of 0.5 sec
// Inputs: number of 0.5 second
// Outputs: None
// Subroutine to wait about 0.5*n_500ms sec
void Delay_N_HalfSec(uint8_t n_500ms) {
	for (uint32_t i=n_500ms*HALF_SEC;i>0;i--){}	
}
