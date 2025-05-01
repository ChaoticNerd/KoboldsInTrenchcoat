// Sensor.c
// Line Following Robot Starter program 
// Created by TYLOR FRANCA 
// February 2025
// CECS 497 SPRING 2025
// California State University, Long Beach
// Modified by Min He and Mark Joseph Austria
// 4/25/2025

// DONE

#include "tm4c123gh6pm.h"
#include "Sensor.h"
#include "Systick.h"
#include <stdint.h>

// GPIO data register bit address definitions
// Reflectance sensor pin connections:
// PB0:ODD CTRL: right; PB1:EVEN CTRL: left
// TODO: connect to PB and PE data
#define LINESENSOR_CTRL       (*((volatile unsigned long *))0x40024030)
// PE0:Sensor 0; PE1: Sensor 7 
#define LINESENSOR						(*((volatile unsigned long *))0x4002400C)
	
// This wait time should be white color pulse<WAIT_TIME<black color pulse
#define WAIT_TIME         0 		//TODO: Fill WAIT_TIME based on Analog Measurement
#define TEN_MICRO_SEC			10

//TODO: find the right bit positions
#define SENSOR_CTRL_PINS 0x0C   // CTRL ODD: bit 1, CRTL EVEN: bit 0
#define SENSOR_PINS      0x03   // SENSOR 0: bit 0 SENSOR 7: bit 1

uint8_t Sensor_CollectData(void){
	Sensor_Init();
  uint8_t sensor_data;

	// Driving a CTRL pin low for at least 1 ms turns off the emitter LEDs
	SENSOR_CTRL &= ~SENSOR_CTRL_PINS;
	//Delay of 2ms
	// nab a debounce or smthn
	Wait_N_MS(2);

	// Turn on CTRL ODD & EVEN pins to turn on the LED emitters 
	SENSOR_CTRL |= SENSOR_CTRL_PINS;
	
	//Charge capacitors by setting PE0:sensor0, PE1: sensor7 as outputs and output high
	GPIO_PORTE_DIR_R  |= SENSOR_PINS;  // Set PE0,1 as output
	SENSORS |= SENSOR_PINS; // Set PE0,1 HIGH
	//Wait 10 us
	Wait_N_US(10);
	
	//Set PE0,1 as inputs
	GPIO_PORTE_DIR_R &= ~SENSOR_PINS;

	//Wait 'WAIT_TIME' us
	Wait_N_MS(1);
	
	//update sensor_data with the current sensor inputs
	sensor_data = SENSORS;	
  // turn off CTRL ODD & EVEN	pins to turn off the LED emitters to save power
	SENSOR_CTRL &= ~SENSOR_CTRL_PINS;
	//Wait 10ms
	Wait_N_MS(10);
	return sensor_data;
}

// PORTE 1,0 connect to sensor 7/0
// PORTE 3,2 connect to CTRL ODD/EVEN
void Sensor_Init(void){
	// port e init
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4;       //Enable clock for Port B
	while ((SYSCTL_RCGCGPIO_R&SYSCTL_RCGCGPIO_R4)!=SYSCTL_RCGCGPIO_R4){}

  GPIO_PORTE_DIR_R   &= ~SENSOR_PINS;  // Sensor pins are inputs
  GPIO_PORTE_DIR_R   |= ~SENSOR_CTRL_PINS; // sensor pins are outputs
  GPIO_PORTE_AFSEL_R &= ~0x0F;  // Disable alternate function
  GPIO_PORTE_AMSEL_R &= ~0x0F;  // Disable analog
  GPIO_PORTE_DEN_R   |= 0x0F;   // Enable digital
	

}
 
