// SensorTestWithLEDs.c.c
// This progrram is provide code to test reflectance sensors. 
// Created by Min He
// 4/22/2025
// California State University, Long Beach



#include "tm4c123gh6pm.h"
#include "Systick.h"
#include "Sensor.h"
#include "LED.h"

#define NUM_STATES			4
#define BOTH_PWM        0xC0
#define LEFT_PWM				0x80
#define RIGHT_PWM				0x40
#define NO_PWM					0x00

struct State {
	uint8_t COLOR;
	uint8_t PWM;
};

typedef const struct State STyp;

enum movement_States{Forward, F_Left, F_Right, Stop};
enum movement_States S;

STyp FSM[NUM_STATES]={
	{RED,   BOTH_PWM},
	{BLUE,  LEFT_PWM},
	{GREEN, RIGHT_PWM},
	{WHITE, NO_PWM}
};

int main(void){
  Sensor_Init();  
	LED_Init();

  while(1){
		// depends on sensor_data
		// sensor data returns 00 01 10 11 based on sensor input
		switch (Sensor_CollectData()) {
			case 0: 
				LED = RED;  // center
			  break;
			case 1: 			// right
				LED = GREEN;
			  break;
			case 2:				// left
				LED = BLUE;
			  break;
			case 3: 			// lost
				LED = WHITE;
			  break;
			default:
				LED = DARK;
			  break;
		}			
  }
}


