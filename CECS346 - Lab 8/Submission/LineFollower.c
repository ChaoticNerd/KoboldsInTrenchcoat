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
  uint16_t delay;              // Delay in ms
  uint8_t next[4];   					 // next state
};
typedef const struct State State_t;

enum states {Center,Left,Right,Stop};

State_t linefollower_fsm[]={
	{BOTH_PWM,	xxx,	{}},
	{LEFT_PWM,	xxx,	{}},
	{RIGHT_PWM, xxx,	{}},
	{NO_PWM,	xxx,	{}},
};

enum states curr_s;   // Initial state
uint8_t Input;

int main(void){
	uint8_t Input;
	
	System_Init();
	
	//TODO: Fill out starting state
	curr_s =SENSORS;
	
	while (1) {
		//TODO: Fill out FSM Engine	
		MOTORS = FSM[curr_s].motors;
		// CHANGE LED SOMEHOW LED = FSM[curr_s].COLOR;
		Wait_N_MS(10);
		Input = SENSORS;
		//curr_s =;

	}
}

void System_Init(void){
	Sensor_Init();
	SysTick_Init();
	Motor_Init();
}

// SysTick ISR:
// 1. Implement timing control for duty cycle and non-duty cycle
// 2. Output a waveform based on current duty cycle
void SysTick_Handler(void){
	
	// add LED
  if(MOTORS&pwm){   // previous cycle is duty cycle
    NVIC_ST_RELOAD_R = L-1;     // switch to non-duty cycle
  } else{ // previous cycle is non-duty cycle
    NVIC_ST_RELOAD_R = H-1;     // switch to duty cycle
  }
	
	MOTORS ^= pwm; // inverse output for PWM
}