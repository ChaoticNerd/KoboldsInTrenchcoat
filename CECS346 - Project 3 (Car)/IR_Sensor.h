// IR_Sensor.h
// IR Sensor for Line Following Robot and Gate Project
// California State University, Long Beach
// CECS 346
// by Natasha Kho, Justin Narciso, Hanna Estrada, William Grefaldeo
// 5/2/2025

#define IR_Sensor_H

#include <stdint.h> // C99 data types
#include <stdbool.h> // boolean data
#include "tm4c123gh6pm.h"

// Sensor bit address defenition for PD6
#define IR_SENSOR 			(*((volatile uint32_t *)0x40007100))		// PD6 for sensor address
	
#define IR_SENSOR_MASK 0x40						  // PD6

#define PORTD_INT_PRI  1U							//PD priority 1
#define PORTD_PRI_BITS 0xE0000000			//Priority Bits 31-29
#define NVIC_EN0_PORTD 0x08						//Value to enable PD intterupt
#define PD_ICR_VAL 		 0x40						//intterupt clear register value

// External Function Prototypes (external functions from startup.s)
extern void DisableInterrupts(void); // Disable interrupts
extern void EnableInterrupts(void);  // Enable interrupts
extern void WaitForInterrupt(void);  // Go to low power mode while waiting for the next interrupt

void IR_Sensor_Init(void);  		// Initialize edge trigger interrupt for PD6


