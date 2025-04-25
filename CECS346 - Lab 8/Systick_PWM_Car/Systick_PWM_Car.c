/////////////////////////////////////////////////////////////////////////////
// Systick_PWM_Car.c
// Example project for using Systick timer to generate PWM signals to control 
// the movement of a DC motor Car.
// Description: 
// Hardware connections: This program works for Romi chassis 
// PB67 for motor PWM signals: PB6 - Left DC Motor, PB7 - Right DC Motor
// PB5432 for motor directions: PB54 - left DC Motor, PB32 - right DC Motor
// By Min He, 4/20/2025
/////////////////////////////////////////////////////////////////////////////

//////////////////////1. Pre-processor Directives Section////////////////////
#include "tm4c123gh6pm.h"
#include "SystickPWM.h"
#include <stdint.h>

/////////////////////////////////////////////////////////////////////////////

//////////////////////2. Declarations Section////////////////////////////////

////////// Local Global Variables //////////

// Function Prototype
// Subroutine to wait 3 sec
// Inputs: None
// Outputs: None
void Delay(void);

#define THREE_SEC 2300000UL

//////////////////////3. Subroutines Section/////////////////////////////////
int main(void){
	enum DIR dir;
	
  Motor_Init(MID);  // medium speed: 50% duty cycle
//  Motor_Init(SLOW); // slow speed: 30% duty cycle
//  Motor_Init(FAST); // fast speed: 90% duty cycle
	

	// Move the motor in the following order defined in enum DIR:
	// FORWARD_STRAIGHT, BACKWARD_STRAIGHT,FORWARD_LEFT,FORWARD_RIGHT,
	// BACKWARD_LEFT,BACKWARD_RIGHT,PIVOT_CCW, PIVOT_CW
	for (dir = FORWARD_STRAIGHT;dir<=PIVOT_CW;dir++) {
		Motor_Start(dir);
		Delay();

		// stop
		Motor_Stop(); // stop both wheels
		Delay();
	}
	
  while(1){}
}

// Subroutine to wait about 3 sec
// Inputs: None
// Outputs: None
void Delay(void){
	uint32_t time;

	time = THREE_SEC;  // timing control for 3 second
	while(time){
		time--;
  }
}
