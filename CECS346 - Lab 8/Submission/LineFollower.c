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
#include <stdint.h>

#define NUM_STATES	4
#define DELAY_MOVING	5
#define DELAY_STOP	10

// Function prototypes
void System_Init(void);

struct State {
  uint8_t motors;              // controls motor power supply
  uint16_t delays;              // Delay in ms
  uint8_t next[NUM_STATES];   			// next state
};
typedef const struct State State_t;

// Center = move forward, left = turn left, right = turn right, stop = no movement
enum states {Center,Left,Right,Stop};

State_t linefollower_fsm[NUM_STATES]={
	{FORWARD,			DELAY_MOVING,	{Center, Left, Right, Stop}},
	{TURN_LEFT,		DELAY_MOVING,	{Center, Left, Right, Stop}},
	{TURN_RIGHT, 	DELAY_MOVING,	{Center, Left, Right, Stop}},
	{STOP,				DELAY_STOP	,	{Center, Left, Right, Stop}},
};

enum states curr_s;   // current state
uint8_t Input;	// define input var

int main(void){
	
	// init system
	System_Init();
	
	// first state based on sensors' first input
	curr_s = SENSORS;
		
	while (1) {
		// set pwm
		MOTORS = linefollower_fsm[curr_s].motors;
		// wait some ms before updating sensor
		Wait_N_MS(linefollower_fsm[curr_s].delays);
		// update input from sensor, then update state
		Input = Sensor_CollectData();
		curr_s = linefollower_fsm[curr_s].next[Input];
	}
}

void System_Init(void){
	Motor_Init();
	Sensor_Init();
	SysTick_Init();
}

