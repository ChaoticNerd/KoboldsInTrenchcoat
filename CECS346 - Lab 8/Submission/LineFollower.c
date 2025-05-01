// LineFollower.c
// Line Following Robot Starter program 
// Created by TYLOR FRANCA 
// February 2025
// CECS 497 SPRING 2025
// California State University, Long Beach
// Modified by Min He and Mark Joseph Austria
// Finished by Natasha Kho, Justin Narciso, Hanna Estrada, William
// 4/25/2025


#include "tm4c123gh6pm.h"
#include "Systick.h"
#include "Motors.h"
#include "Sensor.h"
#include <stdint.h>
// Function prototypes
void System_Init(void);

struct State {
  uint8_t motors;              // controls motor power supply
  uint16_t delays;              // Delay in ms
  uint8_t next[4];   					 // next state
};
typedef const struct State State_t;

enum states {Center,Left,Right,Stop};

State_t linefollower_fsm[4]={
	{FORWARD,			5,	{Center, Left, Right, Stop}},
	{TURN_LEFT,		5,	{Center, Left, Right, Stop}},
	{TURN_RIGHT, 	5,	{Center, Left, Right, Stop}},
	{STOP,				10,	{Center, Left, Right, Stop}},
};

enum states curr_s;   // Initial state
uint8_t Input;

// update sensor data THEN start motor
// this way the systicks do not interfere
int main(void){
	uint8_t Input;
	
	System_Init();
	
	//TODO: Fill out starting state
	curr_s = SENSORS;
		
	while (1) {
		//TODO: Fill out FSM Engine	
		MOTORS = linefollower_fsm[curr_s].motors;
		Motor_Start();
		Wait_N_MS(linefollower_fsm[curr_s].delays);
		// CHANGE LED SOMEHOW LED = FSM[curr_s].COLOR;
		// update sensor value
		Motor_Stop();
		Input = Sensor_CollectData();
		curr_s = Sensor_CollectData();
		Wait_N_US(linefollower_fsm[curr_s].delays);

	}
}

void System_Init(void){
	Motor_Init(0.5 * PERIOD);
	Sensor_Init();
	SysTick_Init();
}

