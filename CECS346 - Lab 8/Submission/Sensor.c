// Sensor.c
// Line Following Robot Starter program 
// Created by TYLOR FRANCA 
// February 2025
// CECS 497 SPRING 2025
// California State University, Long Beach
// Modified by Min He and Mark Joseph Austria
// 4/25/2025

#include "tm4c123gh6pm.h"
#include "Sensor.h"
#include "Systick.h"
#include <stdint.h>

// GPIO data register bit address definitions
// Reflectance sensor pin connections:
// PB0:ODD CTRL: right; PB1:EVEN CTRL: left
// TODO: find the bit addresses
//#define LINESENSOR_CTRL       (*((volatile uint32_t *))?)
// PE0:Sensor 0; PE1: Sensor 7 
//#define LINESENSOR						(*((volatile uint32_t *))?)
	
// This wait time should be white color pulse<WAIT_TIME<black color pulse
#define WAIT_TIME         0 		//TODO: Fill WAIT_TIME based on Analog Measurement
#define TEN_MICRO_SEC			10

//TODO: find the right bit positions
#define SENSOR_CTRL_PINS 0x00   // CTRL ODD: bit 1, CRTL EVEN: bit 0
#define SENSOR_PINS      0x00   // SENSOR 0: bit 0 SENSOR 7: bit 1

uint8_t Sensor_CollectData(void){
  uint8_t sensor_data;

	// Driving a CTRL pin low for at least 1 ms turns off the emitter LEDs
	
	//Delay of 2ms
	

	// Turn on CTRL ODD & EVEN pins to turn on the LED emitters 
	
	
	//Charge capacitors by setting PE0:sensor0, PE1: sensor7 as outputs and output high
	
	//Wait 10 us
	
	
	//Set PE0,1 as inputs

	//Wait 'WAIT_TIME' us
	
	//update sensor_data with the current sensor inputs

  // turn off CTRL ODD & EVEN	pins to turn off the LED emitters to save power
	
	//Wait 10ms
	
	return sensor_data;
}

// PORTE 1,0 connect to sensor 7/0
// PORTE 3,2 connect to CTRL ODD/EVEN
void Sensor_Init(void){
}
 
