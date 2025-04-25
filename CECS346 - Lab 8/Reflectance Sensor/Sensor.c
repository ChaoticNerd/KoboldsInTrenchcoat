// Sensor.c
// Test code for reflectance sensors. 
// Created by TYLOR FRANCA
// Modified by Min He
// 4/22/2025
// California State University, Long Beach


#include "tm4c123gh6pm.h"
#include "Sensor.h"
#include "Systick.h"
#include <stdint.h>

// GPIO data register bit address definitions
#define SENSOR_CTRL       (*((volatile unsigned long *)0x4000500C))
#define SENSORS           (*((volatile unsigned long *)0x4002400C))
	

#define SYSCTL_RCGCGPIO_PORTB	SYSCTL_RCGCGPIO_R1
#define SYSCTL_RCGCGPIO_PORTE SYSCTL_RCGCGPIO_R4
#define SENSOR_CTRL_PINS 0x03   // CTRL ODD: bit 1, CRTL EVEN: bit 0
#define SENSOR_PINS      0x03   // SENSOR 0: bit 0 SENSOR 7: bit 1

uint8_t Sensor_CollectData(void){
  uint8_t sensor_data;

	// Driving a CTRL pin low for at least 1 ms turns off the emitter LEDs
	SENSOR_CTRL  &= ~SENSOR_CTRL_PINS;   
	Wait_N_MS(2);

	// Turn on CTRL ODD & EVEN pins turn on the emitters 
	SENSOR_CTRL |= SENSOR_CTRL_PINS;  // PB0 and PB1 = 1 
	
	//Charge capacitors by setting PE0:sensor0, PE1: sensor7 as outputs and output high
	GPIO_PORTE_DIR_R  |= SENSOR_PINS;  // Set PE0,1 as output
	SENSORS |= SENSOR_PINS; // Set PE0,1 HIGH
	
	//Wait 10 us
	Wait_N_US(10);
	
	//Set PE0,1 as inputs
	GPIO_PORTE_DIR_R &= ~SENSOR_PINS; // Set PE0,1 as input

	Wait_N_US(1000);

	sensor_data = SENSORS;	

  // turn off CTRL ODD & EVEN	
	SENSOR_CTRL &= ~SENSOR_CTRL_PINS;
	Wait_N_MS(10);
	return sensor_data;
}

// PORTE 1,0 connect to sensor 7/0
void PortE_Init(void){
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_PORTE;       //Enable clock for Port B
	while ((SYSCTL_RCGCGPIO_R&SYSCTL_RCGCGPIO_PORTE)!=SYSCTL_RCGCGPIO_PORTE){}

  GPIO_PORTE_DIR_R   &= ~SENSOR_PINS;  // Sensor pins are inputs
  GPIO_PORTE_AFSEL_R &= ~SENSOR_PINS;  // Disable alternate function
  GPIO_PORTE_AMSEL_R &= ~SENSOR_PINS;  // Disable analog
  GPIO_PORTE_DEN_R   |= SENSOR_PINS;   // Enable digital
}

// PORTB 1,0 connect to CTRL ODD/EVEN
void PortB_Init(void){
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_PORTB;       //Enable clock for Port B
	while ((SYSCTL_RCGCGPIO_R&SYSCTL_RCGCGPIO_PORTB)!=SYSCTL_RCGCGPIO_PORTB){}
		
  GPIO_PORTB_DIR_R   |= SENSOR_CTRL_PINS;  // CTRL pins are outputs
  GPIO_PORTB_AFSEL_R &= ~SENSOR_CTRL_PINS;  //Disable alternate function
  GPIO_PORTB_AMSEL_R &= ~SENSOR_CTRL_PINS; //Disable analog
  GPIO_PORTB_DEN_R   |= SENSOR_CTRL_PINS; //Enable digital
}
 
void Sensor_Init(void) {
	PortB_Init();
	PortE_Init();
}