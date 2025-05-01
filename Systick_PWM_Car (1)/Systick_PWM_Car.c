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
#include "LED.h"
#include "Sensor.h"
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

struct State {
  uint8_t colors;              // controls motor power supply
  uint16_t pwm;              // Delay in ms
  uint8_t next[4];   					 // next state
};
typedef const struct State State_t;

enum states {Center,Left,Right,Stop};

State_t linefollower_fsm[]={
	{RED,	FORWARD_STRAIGHT,	{Center,Left,Right,Stop}}, // shoul be go
	{GREEN,	FORWARD_LEFT,	{Center,Left,Right,Stop}}, // 01 turn left
	{BLUE, FORWARD_RIGHT,	{Center,Left,Right,Stop}}, // 10 turn right
	{WHITE,	0,	{Center,Left,Right,Stop}}, // stop
};

enum states curr_s;   // Initial state
uint8_t Input;


//////////////////////3. Subroutines Section/////////////////////////////////
int main(void){
	enum DIR dir;
	Motor_Init(MID);  // medium speed: 50% duty cycle
	Delay();
	LED_Init();
	Sensor_Init();
	LED = WHITE;
	Delay();
	LED = RED;
	Delay();
	
  
//  Motor_Init(SLOW); // slow speed: 30% duty cycle
//  Motor_Init(FAST); // fast speed: 90% duty cycle
	//Motor_Clear();
	//LED = 0x02;

	// Move the motor in the following order defined in enum DIR:
	// FORWARD_STRAIGHT, BACKWARD_STRAIGHT,FORWARD_LEFT,FORWARD_RIGHT,
	// BACKWARD_LEFT,BACKWARD_RIGHT,PIVOT_CCW, PIVOT_CW
	//LED = WHITE;
	/*for (dir = FORWARD_STRAIGHT;dir<FORWARD_LEFT PIVOT_CW;dir++) {
		Motor_Start(dir);
		Delay();

		// stop
		Motor_Stop(); // stop both wheels
		Delay();
	}*/
	//LED_Init();
	//Motor_Stop(); 
	/*Motor_Start(FORWARD_STRAIGHT);
	Delay();

		// stop
	Motor_Stop(); // stop both wheels
	Delay();*/
	dir = FORWARD_STRAIGHT;
	Input = Sensor_CollectData(); // returns int
	curr_s = Center;
	
  while(1){
	/*if (dir == FORWARD_STRAIGHT) {
		dir = BACKWARD_STRAIGHT;
	}
	else if (dir == BACKWARD_STRAIGHT) {
		dir = FORWARD_STRAIGHT;
}*/
	LED = linefollower_fsm[curr_s].colors;
	Input = Sensor_CollectData();
	Delay();
	curr_s = linefollower_fsm[curr_s].next[Input];
	Delay();
	/*LED = WHITE;
	Motor_Start(dir);
	Delay();

		// stop
	LED = RED;
	Motor_Stop(); // stop both wheels*/
	}
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
