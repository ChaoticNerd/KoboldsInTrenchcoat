// DragRace.c
// Starter file for CECS346 Project 2
// By Dr. Min He
// Edited by Justin Narciso, Natasha Kho, William Grefaldeo, Hanna Estrada
// 3/21/2024

#include "tm4c123gh6pm.h"
#include <stdint.h> // C99 data types
#include <stdbool.h> // provides boolean data type
#include "PLL.h"
#include "SysTick.h"
#include "Sensors_Lights.h"

// Function Prototypes
// External functions from startup.s
extern void DisableInterrupts(void); // Disable interrupts
extern void EnableInterrupts(void);  // Enable interrupts
extern void WaitForInterrupt(void);  // Go to low power mode while waiting for the next interrupt

// Functions defined in this file.
void System_Init(void);

// TODO: define bit addresses for two sensors, 8 color lights, and one reset button 
#define SENSORS 											(*((volatile unsigned long *) 0x40004030)) // bit addresses for 2 sensors BUTTONS on A[2:3]
#define LIGHTS              					(*((volatile unsigned long *) 0x400053FC)) // bit addresses for 8 Race Lights PORT B[0:7]
#define RESET                   			(*((volatile unsigned long *) 0x40024010)) // bit address for one reset button: E2

// TODO: define number of states for FSM
#define NUM_STATE            (11U) //U is a constant for unsigned integer 

// TODO: FSM definition
struct State { 
	uint8_t Out;
	uint32_t Time;     // multiple of 0.5 second
	uint8_t Next[NUM_STATE];  
};

typedef const struct State STyp;

// TODO: define reload value for half second
#define HALF_SEC	(8000000U)
#define ONE_SEC		(16000000U)

// TODO: assign a value to all states in Drag Race FSM
// use all upper case for constants defined here
enum DragRace_states {Init,Wait, CD_Y1,CD_Y2,Go,WL,WR,WB,FSL,FSR,FSB};

// TODO: define Outputs for the FSM DONE
#define ALL_ON 	    (~0xFF) // Turns on all the LEDs 
#define ALL_OFF 		(~0x00) // Turns off all the LEDs 
#define YELLOW1_ON	(~0x11) // Turns on 1st set of yellow LEDs 
#define YELLOW2_ON  (~0x22) // Turns on 2nd set of yellow LEDs
#define GREEN_BOTH  (~0x44) // Turns on the green LEDs 
#define GREEN_LEFT  (~0x40) // Turns on left green LED
#define GREEN_RIGHT (~0x04) // Turns on right green LED
#define RED_BOTH    (~0x88) // Turns on both red LEDS
#define RED_LEFT    (~0x80) // Turns on left red LED
#define RED_RIGHT   (~0x08) // Turns on right red LED 

//TODO: Define Drag Race FSM
STyp DragRace_FSM[NUM_STATE] = {
	{ALL_ON, 			ONE_SEC,	{Wait, Wait,	Wait,	Wait}}, 			// Init
	{ALL_OFF, 		HALF_SEC,	{Wait, Wait,	Wait,	CD_Y1}}, 			// Wait
	{YELLOW1_ON, 	HALF_SEC,	{FSB,  FSL, 	FSR, 	CD_Y2}}, 			// CD_Y1
	{YELLOW2_ON, 	HALF_SEC,	{FSB,  FSL, 	FSR, 	Go }},				// CD_Y2
	{GREEN_BOTH, 	HALF_SEC,	{WB, 	 WL, 		WR, 	Go }},				// Go
	{GREEN_LEFT, 	ONE_SEC,	{Wait, Wait,	Wait,	Wait}},				// WL
	{GREEN_RIGHT, ONE_SEC,	{Wait, Wait,	Wait,	Wait}},				// WR
	{GREEN_BOTH, 	ONE_SEC,	{Wait, Wait,	Wait,	Wait}},				// WB
	{RED_LEFT, 		ONE_SEC,	{Wait, Wait,	Wait,	Wait}},				// FSL
	{RED_RIGHT, 	ONE_SEC,	{Wait, Wait,	Wait,	Wait}},				// FSR
	{RED_BOTH, 		ONE_SEC,	{Wait, Wait,	Wait,	Wait}}				// FSB
};

// TODO: define bit positions for left, right and reset buttons
#define RESET_MASK  			(0x01) // bit position for reset button
#define LEFT_SENSOR_MASK  (0x04) // bit position for left sensor
#define RIGHT_SENSOR_MASK (0x08) // bit position for left sensor
	
uint8_t Input;
bool timesup; // default false
bool reset;  // flag to reset the system, set by the reset button located at breadboard, not the launchpad reset button.
uint8_t volatile TEST;
uint8_t volatile check;

	
 int main(void){
  uint8_t S;  // current state index
	
	System_Init();
	 reset = false;
	 timesup = false;
	 S = Init;

  while(1){
    // TODO: reset FSM to its Initial state, reset globals to default values
   	
		Input = SENSORS>>2;	
		while (!reset) {
			// TO Do: take care of FSM outputs and time in state.
			LIGHTS = DragRace_FSM[S].Out;
      SysTick_Start(DragRace_FSM[S].Time);
			check++;

			while((!timesup)&&(!reset)){
				WaitForInterrupt();
				
			}
			
			
			S = DragRace_FSM[S].Next[Input];
			Input = SENSORS>>2;	
			reset = RESET;
		}
		SysTick_Stop();
		S = Init;
  }
}

// Initialize the following hardware modules: PLL, Sensors, Reset button, Lights, SysTick timer
// Initialize the following golbal variables: timesup for FSM state time, reset for reset the whole system
void System_Init(void) {
	DisableInterrupts();
	PLL_Init();
  Sensors_Init(); 
	Reset_Init(); 
	Lights_Init();
	SysTick_Init(); 
  // TODO: reset global variables: timesup, reset, Input 
	timesup = false;
	reset = false;
	Input = SENSORS>>2;
	EnableInterrupts();
}

// Interrupt handler for the two sensors: update Input here 
void GPIOPortA_Handler(void){
	TEST++;
	// simple solution to take care of button debounce: 20ms to 30ms delay
	for (uint32_t i=0;i<160000;i++) {}
	timesup=true;
	
		// NVIC_PRI0_R 5-7 bits 
	if (GPIO_PORTA_RIS_R & 0x08){ //intterupt for R Lane on PA2; PA2 is pressed on during edge trigger
		GPIO_PORTA_ICR_R = 0x08;
	}else if (GPIO_PORTA_RIS_R & 0x04){ //intterupt for L Lane on PA3; PA3 is pressed on during edge trigger
		GPIO_PORTA_ICR_R = 0x04;
	}else if (GPIO_PORTA_RIS_R & 0x0C){ //intterupt for L and R Lane on both PA2 & PA3; PA2 & PA3 is pressed on during edge trigger
		GPIO_PORTA_ICR_R = 0x0C;
	}
}

// Interrupt handler for reset button:  
// update global variable: reset
void GPIOPortE_Handler(void) {
	// simple solution to take care of button debounce: 20ms to 30ms delay
	for (uint32_t i=0;i<160000;i++) {}

	if (GPIO_PORTE_RIS_R & 0x04){ //Reset on PE2 is pressed during trigger
		GPIO_PORTE_ICR_R ;
	}
}

// Systick interrupt handler:
// Stop systick timer and update global variable: timesup 
void SysTick_Handler(void) {
	SysTick_Stop();
	timesup = !timesup;
}

