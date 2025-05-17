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

#define DUTY_1				1
#define DUTY_5				5
#define DUTY_10				10
#define DUTY_30				30
#define DUTY_40				40
#define DUTY_50				50
#define DELAY_LOST		100

// Function prototypes
void System_Init(void);

struct State {
  uint8_t motors;              // controls motor power supply
  double delays;              // Delay in ms
  uint8_t next[NUM_STATES];   			// next state
};
typedef const struct State State_t;

// Center = move forward, left = turn left, right = turn right, stop = no movement
enum states {FORWARD1, FORWARD2, SLIGHT_R1, SLIGHT_R2,SLIGHT_R3,
SLIGHT_L1, SLIGHT_L2,SLIGHT_L3, STOP};
						

/*{Center,Left,Right,Stop}*/

State_t linefollower_fsm[NUM_STATES]={
	{	FORWARD, 		2, 		{ FORWARD2, 	FORWARD2, 	FORWARD2, 	FORWARD2}},
	{	HALT,				8, 		{	FORWARD1, 	SLIGHT_R1, 			SLIGHT_L1, 	STOP	}},

	
	{	FORWARD, 		1, 		{ SLIGHT_R2, 	SLIGHT_R2, 	SLIGHT_R2, 	SLIGHT_R2	}},
	{ TURN_LEFT, 	1, 		{ SLIGHT_R3, 	SLIGHT_R3, 	SLIGHT_R3, 	SLIGHT_R3}},
	{	HALT,				8,		{ FORWARD1, 	SLIGHT_R1, 			SLIGHT_L1, 	STOP}},
	
	{	FORWARD,		1,			{ SLIGHT_L2, 	SLIGHT_L2, 	SLIGHT_L2, 	SLIGHT_L2, 	}},
	{	TURN_RIGHT, 1,			{ SLIGHT_L3, 	SLIGHT_L3, 	SLIGHT_L3, 	SLIGHT_L3 }},
	{	HALT,				8,		{ FORWARD1, 	SLIGHT_R1, 			SLIGHT_L1, 	STOP}},

	{	HALT,				10,		{ FORWARD1, 	SLIGHT_R1, 			SLIGHT_L1, 	STOP	}}
};

enum states curr_s;   // current state
uint8_t Input;	// define input var
bool move_car;
//testing 
uint8_t time; 
// update sensor data THEN start motor
// this way the systicks do not interfere
int main(void){
	// init system
	System_Init();
	
	// first state based on sensors' first input
	//Input = Sensor_CollectData();
	curr_s = FORWARD1;
	move_car = true;
//	while(1){
//		MOTORS = FORWARD;
//		Wait_N_MS(2);
//		MOTORS = HALT;
//		Wait_N_MS(8);
//	}
	while (1) {
		// check if move car is true
		//move_car = true;
		
//		while (move_car) {
			
			MOTORS = linefollower_fsm[curr_s].motors;
			Wait_N_MS(linefollower_fsm[curr_s].delays);		
			time = linefollower_fsm[curr_s].delays;
			Input = Sensor_CollectData();
		  //Input = 0;
			curr_s = linefollower_fsm[curr_s].next[Input];
//		}
//		MOTORS = HALT;
//		while(!move_car){
//			WaitForInterrupt();

//		}
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
	//GPIO_PORTD_RIS_R & IR_SENSOR_MASK		
		GPIO_PORTD_ICR_R = IR_SENSOR_MASK;				//resets interrupt value
		if(!(IR_SENSOR & IR_SENSOR_MASK)){
			move_car = false;
		} else {
			move_car = true;
		}
	}
	