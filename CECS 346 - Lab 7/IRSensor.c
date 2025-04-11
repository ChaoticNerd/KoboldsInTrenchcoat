// IRSensor.c: starter file for CECS346 Lab 7 - Obstacle Avoidance Sensor and Battery Power Supply.
// by Min He
// 4/3/2024

#include <stdint.h> // C99 data types
#include "tm4c123gh6pm.h"

// TODO: LED bit address definition for PF3 and PF1
#define LED       (*((volatile uint32_t *)0x40025028)) 
	
// Sensor bit address defenition for PD6
#define SENSOR 		(*((volatile uint32_t *)0x40007040)) 
	
// position definition for the two onboard LED: red(PF1) and green(PF3)
#define RED  0x02 //PF1
#define GREEN 0x08   // PF3

// External Function Prototypes (external functions from startup.s)
extern void DisableInterrupts(void); // Disable interrupts
extern void EnableInterrupts(void);  // Enable interrupts
extern void WaitForInterrupt(void);  // Go to low power mode while waiting for the next interrupt

// Function Prototypes for functions defined in this file.
void Sensor_Init(void);  // Initialize edge trigger interrupt for PD6
void LEDInit(void);     // Initialize Port F LEDs
void Delay10ms(uint32_t n); //Delay function for debounce logic

int main(void){
	DisableInterrupts();
	Sensor_Init();    // initialize GPIO Port F interrupt
  LEDInit();
	EnableInterrupts();
	
	// Initialize onboard LEDs based on current output from the sensor
  // LED = ?
// If statement using Sensor and LED?

	
  while(1){
		WaitForInterrupt();
  }
}

// Initialize Port F LEDs: PF1 & PF3
void LEDInit(void) 
{
	SYSCTL_RCGCGPIO_R |= 0x20; // Activate Port F Clock
	while((SYSCTL_PRGPIO_R & 0x20) == 0); //Wait for Port F to be ready
	
	GPIO_PORTF_DIR_R |= 0x0A;
	GPIO_PORTF_AFSEL_R &= ~0x0A;
	GPIO_PORTF_DEN_R |= 0x0A;
	GPIO_PORTF_AMSEL_R &= ~0x0A;
	GPIO_PORTF_PCTL_R &= 0x0000F0F0;
}

// Initialize edge trigger interrupt for PD6 both edges
void Sensor_Init(void) 
{
	
}

// change LED color whenever an abstacle cross detectable distance
void GPIOPortD_Handler(void) 
{
}
