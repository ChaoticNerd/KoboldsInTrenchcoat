// LineFollower.c
// Line Following Robot Starter program 
// Created by TYLOR FRANCA 
// February 2025
// CECS 497 SPRING 2025
// California State University, Long Beach
// Modified by Min He and Mark Joseph Austria
// Finished by Natasha Kho, Justin Narciso, Hanna Estrada, William Grefaldeo
// 4/25/2025


#include "tm4c123gh6pm.h"
#include "Systick.h"
#include "Motors.h"
#include "Sensor.h"
#include "IR_Sensor.h"
#include <stdint.h>

#define NUM_STATES	17

#define DUTY_5				.5
#define DUTY_10				.10
#define DUTY_30				.30
#define DUTY_40				.40
#define DUTY_50				.50
#define DELAY_LOST		.100

// Function prototypes
void System_Init(void);

struct State {
  uint8_t motors;              // controls motor power supply
  double delays;              // Delay in ms
  uint8_t next[NUM_STATES];   			// next state
};
typedef const struct State State_t;

// Center = move forward, left = turn left, right = turn right, stop = no movement
enum states {FORWARD1, FORWARD2, FORWARD3, SLIGHT_R1, SLIGHT_R2, SLIGHT_R3, RIGHT1, RIGHT2, RIGHT3, 
SLIGHT_L1, SLIGHT_L2, SLIGHT_L3, LEFT1, LEFT2, LEFT3, LOST, STOP};
						

/*{Center,Left,Right,Stop}*/

State_t linefollower_fsm[NUM_STATES]={
	{	FORWARD, 		DUTY_5, 		{ FORWARD2, 	FORWARD2, 	FORWARD2, 	FORWARD2, 	FORWARD2, 	FORWARD2, 	FORWARD2, 	FORWARD2, 	FORWARD2, 	FORWARD2, 	FORWARD2, 	FORWARD2, 	FORWARD2, 	FORWARD2, 	FORWARD2, 	FORWARD2	}},
	{	HALT,				DUTY_5, 		{	FORWARD3, 	FORWARD3, 	FORWARD3, 	FORWARD3, 	FORWARD3, 	FORWARD3, 			FORWARD3, 	FORWARD3,FORWARD3, 	FORWARD3, 			FORWARD3, 	FORWARD3,FORWARD3, 	FORWARD3, 			FORWARD3, 	FORWARD3			}},
	{	HALT,				DUTY_5, 		{	FORWARD1, 	LEFT1, 			SLIGHT_L1, 	SLIGHT_L1, 	SLIGHT_R1, 	FORWARD1, 	FORWARD1, 	LEFT1, 			SLIGHT_R1, 	FORWARD1, 	FORWARD1, 	FORWARD1, 	SLIGHT_R1, 	FORWARD1, 	RIGHT1, 		LOST			}},

	
	{	FORWARD, 		DUTY_5, 		{ SLIGHT_R2, 	SLIGHT_R2, 	SLIGHT_R2, 	SLIGHT_R2, 	SLIGHT_R2, 	SLIGHT_R2, 	SLIGHT_R2, 	SLIGHT_R2, 	SLIGHT_R2, 	SLIGHT_R2, 	SLIGHT_R2, 	SLIGHT_R2, 	SLIGHT_R2, 	SLIGHT_R2, 	SLIGHT_R2, 	SLIGHT_R2	}},
	{ TURN_RIGHT, DUTY_5, 		{ SLIGHT_R3, 	SLIGHT_R3, 	SLIGHT_R3, 	SLIGHT_R3, 	SLIGHT_R3, 	SLIGHT_R3, 	SLIGHT_R3, 	SLIGHT_R3, 	SLIGHT_R3, 	SLIGHT_R3, 	SLIGHT_R3, 	SLIGHT_R3, 	SLIGHT_R3, 	SLIGHT_R3, 	SLIGHT_R3, 	SLIGHT_R3	}},
	{	HALT,				DUTY_40,		{ FORWARD1, 	LEFT1, 			SLIGHT_L1, 	SLIGHT_L1, 	SLIGHT_R1, 	FORWARD1, 	FORWARD1, 	LEFT1, 			SLIGHT_R1, 	FORWARD1, 	FORWARD1, 	FORWARD1, 	SLIGHT_R1, 	FORWARD1, 	RIGHT1, 		LOST			}},

	
	{	FORWARD, 		DUTY_5, 		{ RIGHT2, 		RIGHT2, 		RIGHT2, 		RIGHT2, 		RIGHT2, 		RIGHT2, 		RIGHT2, 		RIGHT2, 		RIGHT2, 		RIGHT2, 		RIGHT2, 		RIGHT2, 		RIGHT2, 		RIGHT2, 		RIGHT2, 		RIGHT2		}},
	{ TURN_RIGHT, DUTY_10, 		{ RIGHT3, 		RIGHT3, 		RIGHT3, 		RIGHT3, 		RIGHT3, 		RIGHT3, 		RIGHT3, 		RIGHT3, 		RIGHT3, 		RIGHT3, 		RIGHT3, 		RIGHT3, 		RIGHT3, 		RIGHT3, 		RIGHT3, 		RIGHT3		}},
	{	HALT,				DUTY_40,		{ FORWARD1, 	LEFT1, 			SLIGHT_L1, 	SLIGHT_L1, 	SLIGHT_R1, 	FORWARD1, 	FORWARD1, 	LEFT1, 			SLIGHT_R1, 	FORWARD1, 	FORWARD1, 	FORWARD1, 	SLIGHT_R1, 	FORWARD1, 	RIGHT1, 		STOP			}},

	
	{	FORWARD,		DUTY_5,			{ SLIGHT_L2, 	SLIGHT_L2, 	SLIGHT_L2, 	SLIGHT_L2, 	SLIGHT_L2, 	SLIGHT_L2, 	SLIGHT_L2, 	SLIGHT_L2, 	SLIGHT_L2, 	SLIGHT_L2, 	SLIGHT_L2, 	SLIGHT_L2, 	SLIGHT_L2, 	SLIGHT_L2, 	SLIGHT_L2, 	SLIGHT_L2	}},
	{	TURN_LEFT,	DUTY_5,			{ SLIGHT_L3, 	SLIGHT_L3, 	SLIGHT_L3, 	SLIGHT_L3, 	SLIGHT_L3, 	SLIGHT_L3, 	SLIGHT_L3, 	SLIGHT_L3, 	SLIGHT_L3, 	SLIGHT_L3, 	SLIGHT_L3, 	SLIGHT_L3, 	SLIGHT_L3, 	SLIGHT_L3, 	SLIGHT_L3, 	SLIGHT_L3 }},
	{	HALT,				DUTY_40,		{ FORWARD1, 	LEFT1, 			SLIGHT_L1, 	SLIGHT_L1, 	SLIGHT_R1, 	FORWARD1, 	FORWARD1, 	LEFT1, 			SLIGHT_R1, 	FORWARD1, 	FORWARD1, 	FORWARD1, 	SLIGHT_R1, 	FORWARD1, 	RIGHT1, 		STOP			}},
	
	{	FORWARD,		DUTY_5,			{ LEFT2, 			LEFT2, 			LEFT2, 			LEFT2, 			LEFT2, 			LEFT2, 			LEFT2, 			LEFT2, 			LEFT2, 			LEFT2, 			LEFT2, 			LEFT2, 			LEFT2, 			LEFT2, 			LEFT2, 			LEFT2			}},
	{	TURN_LEFT,	DUTY_10,		{ LEFT3, 			LEFT3, 			LEFT3, 			LEFT3, 			LEFT3, 			LEFT3, 			LEFT3, 			LEFT3, 			LEFT3, 			LEFT3, 			LEFT3, 			LEFT3, 			LEFT3, 			LEFT3, 			LEFT3, 			LEFT3 		}},
	{	HALT,				DUTY_40,		{	FORWARD1, 	LEFT1, 			SLIGHT_L1, 	SLIGHT_L1, 	SLIGHT_R1, 	FORWARD1, 	FORWARD1, 	LEFT1, 			SLIGHT_R1, 	FORWARD1, 	FORWARD1, 	FORWARD1, 	SLIGHT_R1, 	FORWARD1, 	RIGHT1, 		STOP			}},

	
	{	HALT,				DELAY_LOST,	{ STOP, 			STOP, 			STOP, 			STOP, 			STOP, 			STOP, 			STOP, 			STOP, 			STOP, 			STOP, 			STOP, 			STOP, 			STOP, 			STOP, 			STOP, 			STOP			}},
	{	HALT,				DUTY_30,		{ FORWARD1, 	LEFT1, 			SLIGHT_L1, 	SLIGHT_L1, 	SLIGHT_R1, 	FORWARD1, 	FORWARD1, 	LEFT1, 			SLIGHT_R1, 	FORWARD1, 	FORWARD1, 	FORWARD1, 	SLIGHT_R1, 	FORWARD1, 	RIGHT1, 		LOST			}}
};

enum states curr_s;   // current state
uint8_t Input;	// define input var
bool move_car;

// update sensor data THEN start motor
// this way the systicks do not interfere
int main(void){
	// init system
	System_Init();
	
	// first state based on sensors' first input
	Input = Sensor_CollectData();
	curr_s = linefollower_fsm[curr_s].next[Input];;
	move_car = true;
		
	while (1) {
		// check if move car is true

		while (move_car) {
			Input = Sensor_CollectData();
			Wait_N_MS(linefollower_fsm[curr_s].delays);		
			MOTORS = linefollower_fsm[curr_s].motors;
			curr_s = linefollower_fsm[curr_s].next[Input];
		}
		WaitForInterrupt();
		NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE;
		move_car = true;

	}
}

void System_Init(void){
	DisableInterrupts();				// Disable interrupts in order to setup
	Motor_Init();
	Sensor_Init();
	IR_Sensor_Init();
	SysTick_Init();
	EnableInterrupts();
}

void GPIOPortD_Handler(void) {
	for(uint32_t i = 0; i < 320000; i ++){}	//Interrupt debounce

	if(GPIO_PORTD_RIS_R & IR_SENSOR_MASK){			
		GPIO_PORTD_ICR_R = PD_ICR_VAL;				//resets interrupt value
		move_car = false;
		curr_s = STOP;
	}
	
}
