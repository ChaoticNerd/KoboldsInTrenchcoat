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

// bit addresses in Sensor.h
// define busy wait delays for sensor data collection
#define WAIT_TIME				1000 //changed from 10 to 200 for SENSOR COLLECT
#define	ONE_MILLI_SEC			1
#define TWO_MILLI_SEC			2
#define TEN_MILLI_SEC			10

//TODO: find the right bit positions
#define SENSOR_CTRL_PINS 0x30   // CTRL ODD: bit 2, CRTL EVEN: bit 3
#define SENSOR_PINS      0x03   // SENSOR 0: bit 0 SENSOR 7: bit 1

uint8_t Sensor_CollectData(void){
	Sensor_Init(); // init sensor
  uint8_t sensor_data; // create integer sensor data

	// Driving a CTRL pin low for at least 1 ms turns off the emitter LEDs
	SENSOR_CTRL &= ~SENSOR_CTRL_PINS;
	//Delay of 2ms
	Wait_N_MS(ONE_MILLI_SEC);

	// Turn on CTRL ODD & EVEN pins to turn on the LED emitters 
	SENSOR_CTRL |= SENSOR_CTRL_PINS;
	
	//Charge capacitors by setting PE0:sensor0, PE1: sensor7 as outputs and output high
	GPIO_PORTE_DIR_R  |= SENSOR_PINS;  // Set PE0,1 as output
	SENSORS |= SENSOR_PINS; // Set PE0,1 HIGH CHECK

	//Wait 10 us
	Wait_N_US(10);
	sensor_data = SENSORS;	//update sensor_data with the current sensor inputs
	//Set PE0,1 as inputs
	GPIO_PORTE_DIR_R &= ~SENSOR_PINS;

	//Wait 'WAIT_TIME' us
	Wait_N_MS(WAIT_TIME);
	//sensor_data = SENSORS; // SOMETHING IS WRONG WITH THIS LINE	
	
	//sensor_data = SENSORS;	
  // turn off CTRL ODD & EVEN	pins to turn off the LED emitters to save power
	SENSOR_CTRL &= ~SENSOR_CTRL_PINS;
	//Wait 1ms
	Wait_N_MS(ONE_MILLI_SEC);
	
	// return sensor data to use in LineFollower.c
	return sensor_data;
}

// PORTE 1,0 connect to sensor 7/0
// PORTE 3,2 connect to CTRL ODD/EVEN
// Init Port E
void Sensor_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4;       //Enable clock for Port E
	while ((SYSCTL_RCGCGPIO_R&SYSCTL_RCGCGPIO_R4)!=SYSCTL_RCGCGPIO_R4){} // Wait for clock to be ready

  GPIO_PORTE_DIR_R   &= ~SENSOR_PINS;  // Sensor pins are inputs
  GPIO_PORTE_DIR_R   |= SENSOR_CTRL_PINS; // sensor pins are outputs
  GPIO_PORTE_AFSEL_R &= ~(SENSOR_PINS | SENSOR_CTRL_PINS);  // Disable alternate function
  GPIO_PORTE_AMSEL_R &= ~(SENSOR_PINS | SENSOR_CTRL_PINS);  // Disable analog
  GPIO_PORTE_DEN_R   |= (SENSOR_PINS | SENSOR_CTRL_PINS);   // Enable digital
	

}
 
