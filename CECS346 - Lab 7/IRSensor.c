// IRSensor.c: starter file for CECS346 Lab 7 - Obstacle Avoidance Sensor and Battery Power Supply.
// by Min He
// Updated by Natasha Kho, Justin Narciso, Hanna Estrada, William Grefaldeo
// 4/3/2024

#include <stdint.h> // C99 data types
#include <stdbool.h> // boolean data
#include "tm4c123gh6pm.h"
#include "Servo.h"

// TODO: LED bit address definition for PF3 and PF1
#define LED         (*((volatile uint32_t *)0x40025028))
	
// Sensor bit address defenition for PD6
#define SENSOR 		(*((volatile uint32_t *)0x40007100))
	
// position definition for the two onboard LED: red(PF1) and green(PF3)
#define RED  	0x02	//PF1
#define GREEN   0x08	// PF3

#define SENSOR_MASK 0x40 // PD6

#define PORTD_INT_PRI  3U
#define PORTD_PRI_BITS 0xE0000000
#define NVIC_EN0_PORTD 0x08


// External Function Prototypes (external functions from startup.s)
extern void DisableInterrupts(void); // Disable interrupts
extern void EnableInterrupts(void);  // Enable interrupts
extern void WaitForInterrupt(void);  // Go to low power mode while waiting for the next interrupt

// Function Prototypes for functions defined in this file.
void Sensor_Init(void);  // Initialize edge trigger interrupt for PD6
void Servo_Init(void);   // Initialize servo output pin
void LEDInit(void);     // Initialize Port F LEDs
void Delay10ms(uint32_t n); //Delay function for debounce logic (do we still need this? this isn't in the new file)

enum Servo_Dir {CLOCKWISE, COUNTERCLOCKWISE};

bool move_servo;
enum Servo_Dir dir;

int main(void){
	DisableInterrupts();
	Sensor_Init();    // initialize GPIO Port F interrupt
	Servo_Init();
  LEDInit();
	SysTick_Init();
	EnableInterrupts();
	
	// Initialize the LaunchPad LEDs based on the current sensor output
	// If statement using Sensor and LED?
	

	// TODO: position servo to its original position: middle position
	// i don't think bit shift since it's dependent on like. time?
	// ah ok
	//SERVO = SERVO_START; //Center the PWM signal
	move_servo = false;
	LED = GREEN;
	Drive_Servo(SERVO_START);
  while(1){
		LED = GREEN;
		Drive_Servo(SERVO_START);
		while(!move_servo){
			WaitForInterrupt();
		}
		LED = RED;
		Drive_Servo(SERVO_END);
		while(move_servo){
			WaitForInterrupt();
		}
  }
}

// Initialize Port F LEDs: PF1 & PF3
void LEDInit(void) {
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5; // Activate Port F Clock
	while((SYSCTL_PRGPIO_R & SYSCTL_RCGCGPIO_R5) != SYSCTL_RCGCGPIO_R5); //Wait for Port F to be ready
	
	GPIO_PORTF_DIR_R |= 0x0A;
	GPIO_PORTF_AFSEL_R &= ~0x0A;
	GPIO_PORTF_DEN_R |= 0x0A;
	GPIO_PORTF_AMSEL_R |= 0x0A;
	GPIO_PORTF_PCTL_R &= ~0x0000F0F0;

}

// Initialize edge trigger interrupt for PD6 both edges
void Sensor_Init(void) {
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3;
	while((SYSCTL_PRGPIO_R & SYSCTL_RCGCGPIO_R3) != SYSCTL_RCGCGPIO_R3); //Wait for Port F to be ready

	// Port D6 setup
	GPIO_PORTD_DIR_R		&= ~SENSOR_MASK; 
	GPIO_PORTD_AFSEL_R  &= ~SENSOR_MASK; // look up alt funct of portD
	GPIO_PORTD_DEN_R		|=  SENSOR_MASK;
	GPIO_PORTD_AMSEL_R  |=  SENSOR_MASK;
	GPIO_PORTD_PCTL_R		&= ~SENSOR_MASK;
	
	// Edge Interrupt setup
	// 01X1
	GPIO_PORTD_IS_R 	&= ~SENSOR_MASK;
	GPIO_PORTD_IBE_R 	|= SENSOR_MASK; 
	GPIO_PORTD_IM_R 	|= SENSOR_MASK;

	// Priority 3
	NVIC_PRI0_R = (NVIC_PRI0_R&~PORTD_PRI_BITS)|PORTD_INT_PRI<<29;
	NVIC_EN0_R |= NVIC_EN0_PORTD;
}

//this is not on the new code, do we still need this?
// change LED color whenever an abstacle cross detectable distance
void GPIOPortD_Handler(void) {
	for(uint32_t i = 0; i < 160000; i ++){}
	if(GPIO_PORTD_RIS_R & SENSOR_MASK){
		GPIO_PORTD_ICR_R = 0x40;
		move_servo = !move_servo;
	}
}

// Interrupt service routine for sensor interrupt
// change LED color whenever an abstacle cross detectable distance
// Set move_servo and determine the moving direction: dir 
//void GPIOPortD_Handler(void) 
//{
//}
