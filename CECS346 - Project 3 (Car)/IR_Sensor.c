// IRSensor.c: file for CECS346 - Project 3: Obstacle Avoidance Sensor
// by Natasha Kho, Justin Narciso, Hanna Estrada, William Grefaldeo
// 5/2/2024


#include <stdint.h> // C99 data types
#include <stdbool.h> // boolean data
#include "tm4c123gh6pm.h"
#include "IR_Sensor.h"


// Initialize edge trigger interrupt for PD6 both edges
void IR_Sensor_Init(void) {
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3;
	while((SYSCTL_PRGPIO_R & SYSCTL_RCGCGPIO_R3) != SYSCTL_RCGCGPIO_R3); //Wait for Port D to be ready

	// Port D6 setup
	GPIO_PORTD_DIR_R		&= ~IR_SENSOR_MASK; // Set PD6 as input
	GPIO_PORTD_AFSEL_R  &= ~IR_SENSOR_MASK; // no alt function for PORT D
	GPIO_PORTD_DEN_R		|=  IR_SENSOR_MASK; // enable digital Pin on PD6
	GPIO_PORTD_AMSEL_R  &= ~IR_SENSOR_MASK; // PD6 disable enable function
	GPIO_PORTD_PCTL_R		&= ~IR_SENSOR_MASK; // Clear PD6 PCTL
	
	// Edge Interrupt setup
	GPIO_PORTD_IS_R 	&= ~IR_SENSOR_MASK; 	 // interrupt edge triggered
	/*GPIO_PORTD_IS_R 	|= IR_SENSOR_MASK;		// interrupt level triggered
	GPIO_PORTD_IBE_R &= ~IR_SENSOR_MASK;        // check interrupt event iev
	GPIO_PORTD_IEV_R |= IR_SENSOR_MASK;		// high level trigger*/
	GPIO_PORTD_IBE_R 	|= IR_SENSOR_MASK;  	 // Interrupt both Edge	
	GPIO_PORTD_ICR_R = PD_ICR_VAL;				//resets interrupt value
	GPIO_PORTD_IM_R 	|= IR_SENSOR_MASK;		 // set interrupt pin to be sent the interrupt controller

	// Priority 1
	NVIC_PRI0_R = (NVIC_PRI0_R&~PORTD_PRI_BITS)|PORTD_INT_PRI;	//clear priority bits and set priority to 1
	NVIC_EN0_R |= NVIC_EN0_PORTD;																		//Interrupt enable
}

