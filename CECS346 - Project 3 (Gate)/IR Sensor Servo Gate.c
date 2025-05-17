// IRSensor.c: starter file for CECS346 Lab 7 - Obstacle Avoidance Sensor and Battery Power Supply.
// by Min He
// Updated by Natasha Kho, Justin Narciso, Hanna Estrada, William Grefaldeo
// 4/3/2024

#include <stdint.h> // C99 data types
#include <stdbool.h> // boolean data
#include "Servo.h"
#include "tm4c123gh6pm.h"

// TODO: LED bit address definition for PF3 and PF1
#define LED         (*((volatile uint32_t *)0x40025028))		// LED value for PF3 and PF1 the onboard LEDs
	
// Sensor bit address defenition for PD6
#define SENSOR 			(*((volatile uint32_t *)0x40007100))		// PD6 for sensor address

// Servo bit address definition for PB6
#define SERVO 	(*((volatile uint32_t *)0x40005300))  // use PB6-7 for Servo
	
// position definition for the two onboard LED: red(PF1) and green(PF3)
#define RED  		0x02	 								// PF1
#define GREEN   0x08		 							// PF3

#define SENSOR_MASK 0x40						  // PD6

#define SERVO_BIT_LEFT 	0x40								// PB6 || left
#define SERVO_BIT_RIGHT 0x80								// PB7 || right
#define SERVO_BIT_MASK 	0xC0								// PB67|| setup purposes
#define SERVO_PCTL_MASK 0xFF000000					// PB6-7 PCTL

#define PORTD_INT_PRI  3U							//PD priority 3
#define PORTD_PRI_BITS 0xE0000000			//Priority Bits 31-29
#define NVIC_EN0_PORTD 0x08						//Value to enable PD intterupt
#define PD_ICR_VAL 		 0x40						//intterupt clear register value

#define PORTF_MASK		0x0A						//PF Mask value
#define PORTF_PCTL		0x0000F0F0			//PCTL value for Port F

#define PRI3_TOP3_BITS_RESET	0x1FFFFFFFF 	// used to clear all priority bits of systick
#define PRI3_TOP3_BITS_SET		0x800000000		// used to set systick priority as 4
#define Duty_Cycle_toggle 		1 						// Duty cycle to toggle value
#define Systick_reset 				0							// ValuE to set count to 0

// high(duty cycle) and low(non-duty cycle) reload values
static uint32_t H;
static uint32_t L;	

// External Function Prototypes (external functions from startup.s)
extern void DisableInterrupts(void); // Disable interrupts
extern void EnableInterrupts(void);  // Enable interrupts
extern void WaitForInterrupt(void);  // Go to low power mode while waiting for the next interrupt

// Function Prototypes for functions defined in this file.
void Sensor_Init(void);  		// Initialize edge trigger interrupt for PD6
void Servo_Init(void);   		// Initialize servo output pin
void LEDInit(void);     		// Initialize Port F LEDs
void SysTick_Init(void);		// Initalize SysTick 
void Open_Door(void);
void Close_Door(void);

enum Servo_Dir {CLOCKWISE, COUNTERCLOCKWISE}; 

bool move_servo;							//the value to determine if servo moves
enum Servo_Dir dir;						//
bool detected;

int main(void){
	DisableInterrupts();				// Disable interrupts in order to setup
	Sensor_Init();    					// initialize GPIO Port F interrupt
	Servo_Init();								// Initialize The Servo
  LEDInit();									// Initialize The LED functions
	SysTick_Init();							// Initialize the Systick to allow CLK to function
	EnableInterrupts();					// Renable Interrupts as set up has ended and begin functions of main
	
	move_servo = false;					// servo not moving at start
	LED = GREEN;								// LED initially set to green
	
	detected = false;
	
  while(1){
		LED = GREEN;	// set LED to green
		Close_Door();
		while(!detected){
			WaitForInterrupt();
		}
		LED = RED;								// set LED to red
		Open_Door();
		while(detected){
			WaitForInterrupt();
		}
		for(int i = 0; i < 0xFF; i++){}
  }
}

// Initialize Port F LEDs: PF1 & PF3
void LEDInit(void) {
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5; 														 // Activate Port F Clock
	while((SYSCTL_PRGPIO_R & SYSCTL_RCGCGPIO_R5) != SYSCTL_RCGCGPIO_R5); //Wait for Port F to be ready
	
	GPIO_PORTF_DIR_R |= PORTF_MASK;					// set PF1 and PF3 to output
	GPIO_PORTF_AFSEL_R &= ~PORTF_MASK;			// Not Alternate function for PORT F
	GPIO_PORTF_DEN_R |= PORTF_MASK;					// Enable digital PINS for PF3 & PF1
	GPIO_PORTF_AMSEL_R &= ~PORTF_MASK;			// disable analog functions
	GPIO_PORTF_PCTL_R &= ~PORTF_PCTL;				// Clear PortF PCTL
}

// Initialize edge trigger interrupt for PD6 both edges
void Sensor_Init(void) {
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3;
	while((SYSCTL_PRGPIO_R & SYSCTL_RCGCGPIO_R3) != SYSCTL_RCGCGPIO_R3); //Wait for Port F to be ready

	// Port D6 setup
	GPIO_PORTD_DIR_R		&= ~SENSOR_MASK; // Set PD6 as input
	GPIO_PORTD_AFSEL_R  &= ~SENSOR_MASK; // no alt function for PORT D
	GPIO_PORTD_DEN_R		|=  SENSOR_MASK; // enable digital Pin on PD6
	GPIO_PORTD_AMSEL_R  &= ~SENSOR_MASK; // PD6 disable enable function
	GPIO_PORTD_PCTL_R		&= ~SENSOR_MASK; // Clear PD6 PCTL
	
	// Edge Interrupt setup
	GPIO_PORTD_IS_R 	&= ~SENSOR_MASK; 	 // interrupt edge triggered
	GPIO_PORTD_IBE_R 	|= SENSOR_MASK;  	 // Interrupt both Edge
	GPIO_PORTD_IM_R 	|= SENSOR_MASK;		 // set interrupt pin to be sent the interrupt controller
	//GPIO_PORTD_IEV_R  	|= SENSOR_MASK;		 // high level trigger

	// Priority 3
	NVIC_PRI0_R = (NVIC_PRI0_R&~PORTD_PRI_BITS)|PORTD_INT_PRI<<29;	//clear priority bits and set priority to 3
	NVIC_EN0_R |= NVIC_EN0_PORTD;									//Interrupt enable
}

void Servo_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;																//Activate GPIOB Clock
	while((SYSCTL_RCGCGPIO_R & SYSCTL_RCGCGPIO_R1) != SYSCTL_RCGCGPIO_R1);	
	
	GPIO_PORTB_AFSEL_R 	&= ~SERVO_BIT_MASK;							// Disable Alternate Function
	GPIO_PORTB_PCTL_R 	&= ~SERVO_PCTL_MASK;				    // set to GPIO
	GPIO_PORTB_DIR_R 		|= SERVO_BIT_MASK;							// set to output pin
	GPIO_PORTB_AMSEL_R 	&= ~SERVO_BIT_MASK;							// Disable Analog Function
	GPIO_PORTB_DEN_R 		|= SERVO_BIT_MASK;							// Enable Digital I/O
}

void SysTick_Init(void){
  NVIC_ST_CTRL_R &= ~NVIC_ST_CTRL_ENABLE;    																			// disable SysTick during setup
	NVIC_ST_CTRL_R |= NVIC_ST_CTRL_CLK_SRC;										// enable SysTick clock source
}

void GPIOPortD_Handler(void) {
	for(uint32_t i = 0; i < 160000; i ++){}	//Interrupt debounce
	if(GPIO_PORTD_RIS_R & SENSOR_MASK){			
		GPIO_PORTD_ICR_R = PD_ICR_VAL;				//resets interrupt value
		detected = !detected;
		move_servo = !move_servo;
	}
}

void Open_Door(void){
	NVIC_ST_CTRL_R &= ~NVIC_ST_CTRL_ENABLE;
	for(int i = 0; i < 10; i++){
		
		// ****************   HIGH CYCLE   **************** //
		// LEFT ON
		SERVO |= SERVO_BIT_MASK;
		NVIC_ST_RELOAD_R = SERVO_CW_90; // AT CENTER FOR BOTH
		// WAITING
		NVIC_ST_CURRENT_R = 0; // clear countdown counter
		NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE; // enable SysTick timer
		while ((NVIC_ST_CTRL_R & NVIC_ST_CTRL_COUNT) == 0); // busy wait timer
		NVIC_ST_CTRL_R &= ~NVIC_ST_CTRL_ENABLE; // disable SysTick again to set up
		
		SERVO &= ~SERVO_BIT_LEFT; // LEFT OFF
		
		// ****************   LOW CYCLE   **************** //
		// NVIC_ST_RELOAD_R = SERVO_CENTER + SERVO_CW_90; // CW 90    THIS MOVES CCW 90 FOR SOME REASON
		NVIC_ST_RELOAD_R = SERVO_CCW_90 - SERVO_CW_90;
		NVIC_ST_CURRENT_R = 0; // clear countdown counter
		NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE; // enable SysTick timer
		while ((NVIC_ST_CTRL_R & NVIC_ST_CTRL_COUNT) == 0); // busy wait timer
		NVIC_ST_CTRL_R &= ~NVIC_ST_CTRL_ENABLE; // disable SysTick again to set up
		
		SERVO &= ~SERVO_BIT_RIGHT; // LEFT OFF
		// ****************   LOW CYCLE   **************** //    NEW ADDITION
		NVIC_ST_RELOAD_R = SERVO_PERIOD - SERVO_CW_90 - SERVO_CW_90;
		NVIC_ST_CURRENT_R = 0; // clear countdown counter
		NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE; // enable SysTick timer
		while ((NVIC_ST_CTRL_R & NVIC_ST_CTRL_COUNT) == 0); // busy wait timer
		NVIC_ST_CTRL_R &= ~NVIC_ST_CTRL_ENABLE; // disable SysTick again to set up
		
		/*
		// ****************   HIGH CYCLE   **************** //
		SERVO |= SERVO_BIT_RIGHT;
		NVIC_ST_RELOAD_R = CENTER; // CCW 90 PERIOD - CW90 -CCW90
		NVIC_ST_CURRENT_R = 0; // clear countdown counter
		NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE; // enable SysTick timer
		while ((NVIC_ST_CTRL_R & NVIC_ST_CTRL_COUNT) == 0); // busy wait timer
		NVIC_ST_CTRL_R &= ~NVIC_ST_CTRL_ENABLE; // disable SysTick again to set up
		
		// ****************   LOW CYCLE   **************** //
		
		NVIC_ST_RELOAD_R = CENTER + CCW90; // CW 90
		NVIC_ST_CURRENT_R = 0; // clear countdown counter
		NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE; // enable SysTick timer
		while ((NVIC_ST_CTRL_R & NVIC_ST_CTRL_COUNT) == 0); // busy wait timer
		NVIC_ST_CTRL_R &= ~NVIC_ST_CTRL_ENABLE; // disable SysTick again to set up
		SERVO &= ~SERVO_BIT_RIGHT;
		*/
	}
}
	
void Close_Door(void){
	NVIC_ST_RELOAD_R = 16000000;
	NVIC_ST_CURRENT_R = 0; // clear countdown counter
	NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE; // enable SysTick timer
	while ((NVIC_ST_CTRL_R & NVIC_ST_CTRL_COUNT) == 0); // busy wait timer
	NVIC_ST_CTRL_R &= ~NVIC_ST_CTRL_ENABLE; // disable SysTick again to set up
	
	for(int i = 0; i < 5; i++){
	// TURN BOTH ON
		SERVO |= SERVO_BIT_MASK;
		NVIC_ST_RELOAD_R = SERVO_CENTER; // AT CENTER FOR BOTH
		// WAITING
		NVIC_ST_CURRENT_R = 0; // clear countdown counter
		NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE; // enable SysTick timer
		while ((NVIC_ST_CTRL_R & NVIC_ST_CTRL_COUNT) == 0); // busy wait timer
		NVIC_ST_CTRL_R &= ~NVIC_ST_CTRL_ENABLE; // disable SysTick again to set up
		
	// TURN BOTH OFF
		SERVO &= ~SERVO_BIT_MASK;
		NVIC_ST_RELOAD_R = SERVO_PERIOD - SERVO_CENTER; // AT CENTER FOR BOTH
		// WAITING
		NVIC_ST_CURRENT_R = 0; // clear countdown counter
		NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE; // enable SysTick timer
		while ((NVIC_ST_CTRL_R & NVIC_ST_CTRL_COUNT) == 0); // busy wait timer
		NVIC_ST_CTRL_R &= ~NVIC_ST_CTRL_ENABLE; // disable SysTick again to set up
	}
}