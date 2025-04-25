// Sensor.c
// Sensor Decay Test Code 
// Created by TYLOR FRANCA 
// February 2025
// CECS 497 SPRING 2025
// California State University, Long Beach
// Modified by Min He

#include "tm4c123gh6pm.h"
#include "Sensor.h"
#include "Systick.h"

// PORTE pin assignments
// PE0: Sensor 0  
// PE1: CTRL even  
// PE2: Sensor 0 read back digital value 
void Sensor_Init(void){
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4;       //Enable clock for Port E
	while ((SYSCTL_RCGCGPIO_R&SYSCTL_RCGCGPIO_R4)!=SYSCTL_RCGCGPIO_R4){}
	
	// set sensor CTRL even and digital sensor read back as outputs
  GPIO_PORTE_DIR_R   |= CONTRL_EVEN_MASK|SENSOR0_DIGITAL_MASK;  
	// Set PE0 as input, physical sensor
	GPIO_PORTE_DIR_R   &= ~SENSOR0_MASK;
		
	// these 3 are the same as ever
  GPIO_PORTE_AFSEL_R &= ~(CONTRL_EVEN_MASK|SENSOR0_DIGITAL_MASK|SENSOR0_MASK); // Disable alternate function for PE0,1,2
  GPIO_PORTE_AMSEL_R &= ~(CONTRL_EVEN_MASK|SENSOR0_DIGITAL_MASK|SENSOR0_MASK); //Disable analog for PE0,1,2
  GPIO_PORTE_DEN_R   |= CONTRL_EVEN_MASK|SENSOR0_DIGITAL_MASK|SENSOR0_MASK;  //Enable digital for PE0,1,2
}

void CollectSensorData(void){
	// Turn on EVEN CTRL (PE1 high)
	CTRL_EVEN |= CONTRL_EVEN_MASK;  
	
	//Charge capacitor by setting SENSOR0 as output and output high
	SENSOR0_DIR_R |= SENSOR0_MASK;  // Set PE0 as output
	SENSOR0 |= SENSOR0_MASK; // Set PE0 HIGH

	//Wait 10 µs
	Wait_N_US(10);

	//in short: charge caps then set pe0 as an input
	//Set PE0 as input
	SENSOR0_DIR_R &= ~SENSOR0_MASK; // Set PE0 as input

	// Repeatedly reads sensor input and send the value to a digital output pin
	// make sure time is long enough to capture black surface
	// aka duration for how long input can be collected
	for(int i = 0; i < 10000; i++){
		SENSOR0_DIGITAL = SENSOR0<<2;
  }
	
	// what does even ctrl do
	// Turn off EVEN CTRL (PE1 low)
	CTRL_EVEN &= ~CONTRL_EVEN_MASK;
		
	Wait_N_MS(10);
}