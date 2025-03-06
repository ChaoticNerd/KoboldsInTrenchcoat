// CECS346 Lab 4: SysTick
// Team members: Justin Narciso, Natasha Kho, Hanna Estrada, William Grefaldeo
// Lab description: Creating a properly timed delay to control the traffic light using SysTick Timer

// Hardware Design
// 1)	Port E will be used to control 4 LEDs: white(PE3), red (PE2), yellow (PE1), green (PE0).
// 2)	Port A will be used for the two switches: sw1 (PA2), sw2 (PA3)

#include <stdint.h>   // for data type alias
#include "Lab4SysTick.h"

// Registers for switches
// Complete the following register definitions
#define SENSOR									(*((volatile uint32_t *)0x40004030)) // bit addresses for the two switches/Sensors: bits 2&3 
#define GPIO_PORTA_DATA_R       (*((volatile uint32_t *)0x400043FC)) // calls all pins in port to be able to read/write
#define GPIO_PORTA_DIR_R        (*((volatile uint32_t *)0x40004400)) 
#define GPIO_PORTA_AFSEL_R      (*((volatile uint32_t *)0x40004420)) 
#define GPIO_PORTA_DEN_R        (*((volatile uint32_t *)0x4000451C)) 
#define GPIO_PORTA_AMSEL_R      (*((volatile uint32_t *)0x40004528)) 
#define GPIO_PORTA_PCTL_R       (*((volatile uint32_t *)0x4000452C)) 
#define GPIO_PORTA_PDR_R        (*((volatile uint32_t *)0x40004514)) // for resistor

//// Registers for LEDs
#define LIGHT                 	(*((volatile uint32_t *)0x4002403C)) // bit addresses for the four LEDs (0-3)
#define GPIO_PORTE_DIR_R        (*((volatile uint32_t *)0x40024400)) 
#define GPIO_PORTE_AFSEL_R      (*((volatile uint32_t *)0x40024420)) 
#define GPIO_PORTE_DEN_R        (*((volatile uint32_t *)0x4002451C)) 
#define GPIO_PORTE_AMSEL_R      (*((volatile uint32_t *)0x40024528))
#define GPIO_PORTE_PCTL_R       (*((volatile uint32_t *)0x4002452C))
#define SYSCTL_RCGC2_R          (*((volatile uint32_t *)0x400FE108)) // activate internal clock
	
// Constants definitions
#define APORT_23         				0x0C
#define APORT_CTL        				0x0000FF00
#define EPORT_0123       				0x0F
#define EPORT_CTL        				0x0000FFFF
#define SYSCTL_RCGC2_GPIOE    	0x00000010    // port E Clock Gating Control
#define SYSCTL_RCGC2_GPIOA    	0x00000001    // port A Clock Gating Control
#define HALF_SEC 								1   					// Half second delay (0.5 x 1)
#define ONE_SEC 								2							// One second delay (0.5 x 2)
#define TWO_SEC									4							// Two second delay (0.5 x 4)
#define NO_DELAY        				0             // Zero second delay (0.5 x 0)

#define LED_GREEN       				0x01          // green LED (port E0)
#define LED_YELLOW 							0x02          // yellow LED (port E1)
#define LED_REDWHITE_0 					0x0C          // red and white LED (port E2, E3)
#define LED_RED 								0x04          // red LED (port E0)
#define LED_REDWHITE_1 					0x0C          // red and white LED (port E2, E3)
#define SENSOR_SHIFT 						2             // shift SENSOR to right by 2 (since we're using port A2 and A3)     

#define NUM_STATES 							8             // number of states (using 5/8)

// define each function
void Delay(uint8_t n);
void Light_Init(void);
void Sensor_Init(void);

// FSM state data structure
// Declares the data type for each element in array
struct State {
	// components at [X][0] represent Out, [X][1] represent Duration of State, [X][2] represent Next State
  uint8_t Out; 
  uint8_t Time;  
  uint8_t Next[NUM_STATES];
}; 

typedef const struct State STyp;		// never changes

// Constants definitions
// states connected to each output
enum my_states {GREEN, YELLOW, REDWHITE_0, RED, REDWHITE_1, NULL_0, NULL_1, NULL_2};

// Output pins are: 3(white), 2(red), 1(yellow), 0(green)
// Input pins are: 1:sw2, 0:sw1 
// Declares and initializes values to Finite State Machine
// in order: next state for next input 000 001 010 011 100 101 110 111
// note: 100 101 110 111 are same states as 000 001 010 011 due to having same last 2 bits
// ** based on state diagram/table
// no button means stay at current state

STyp FSM[NUM_STATES]={
	{LED_GREEN, 						TWO_SEC, 	{GREEN,      GREEN,      YELLOW,     YELLOW,     GREEN,      GREEN,      YELLOW,     YELLOW}},
	{LED_YELLOW, 						ONE_SEC, 	{REDWHITE_0, REDWHITE_0, REDWHITE_0, REDWHITE_0, REDWHITE_0, REDWHITE_0, REDWHITE_0, REDWHITE_0}},
	{LED_REDWHITE_0,        TWO_SEC, 	{REDWHITE_0, RED,        REDWHITE_0, RED,        REDWHITE_0, RED,        REDWHITE_0, RED}},
	{LED_RED,               HALF_SEC, {REDWHITE_1, REDWHITE_1, REDWHITE_1, REDWHITE_1, REDWHITE_1, REDWHITE_1, REDWHITE_1, REDWHITE_1}},
	{LED_REDWHITE_1, 			  HALF_SEC, {GREEN,      GREEN,      GREEN,      GREEN,      GREEN,      GREEN,      GREEN,      GREEN}},
  {NULL_0,                NO_DELAY, {GREEN,      GREEN,      GREEN,      GREEN,      GREEN,      GREEN,      GREEN,      GREEN}},
  {NULL_1,                NO_DELAY, {GREEN,      GREEN,      GREEN,      GREEN,      GREEN,      GREEN,      GREEN,      GREEN}},
	{NULL_2,                NO_DELAY, {GREEN,      GREEN,      GREEN,      GREEN,      GREEN,      GREEN,      GREEN,      GREEN}}
};
//yee
int main(void){ 
  uint8_t S;  // index to the current state 
  uint8_t Input; 
	SysTick_Init();
	
	Light_Init();
	Sensor_Init();
  S = GREEN;                     // FSM start with green  
    
  while(1){
		// output LED color 
		LIGHT = FSM[S].Out;
		// activate delay function with correct delay duration from current state
		Wait_N_Half_Sec(FSM[S].Time);
		
		// 00001100
		// --> 11110011 (when pressed)
		// Input = Current Button press value, AND SENSOR with APORT_23 to clear other bits
		Input = (((~SENSOR & APORT_23) >> SENSOR_SHIFT));
		// Update State
		S = FSM[S].Next[Input];
  }
}

void Sensor_Init(void){
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA;       // Activate Port A clocks
	while ((SYSCTL_RCGC2_R & SYSCTL_RCGC2_GPIOA)!= SYSCTL_RCGC2_GPIOA) {} // wait for clock to be active
		
	GPIO_PORTA_AMSEL_R &= ~APORT_23;  // Disable analog function on PA3-2
  GPIO_PORTA_PCTL_R &= ~APORT_CTL; // Enable regular GPIO
  GPIO_PORTA_DIR_R  &= ~APORT_23;  // Inputs on PA3-2
  GPIO_PORTA_AFSEL_R &= ~APORT_23; // Regular function on PA3-2
  GPIO_PORTA_DEN_R |= APORT_23;   // Enable digital signals on PA3-2
	// GPIO_PORTA_PDR_R |= APORT_23;  // Enable pull-down resistors for PA3-2 (Optional)
}

void Light_Init(void){
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOE;      // Activate Port E clocks
	while ((SYSCTL_RCGC2_R & SYSCTL_RCGC2_GPIOE)!= SYSCTL_RCGC2_GPIOE) {} // wait for clock to be active
		
  GPIO_PORTE_AMSEL_R &= ~EPORT_0123;// Disable analog function on PE3-0
  GPIO_PORTE_PCTL_R &= ~EPORT_CTL;// Enable regular GPIO
  GPIO_PORTE_DIR_R  |= EPORT_0123;  // Outputs on PE3-0
  GPIO_PORTE_AFSEL_R &= ~EPORT_0123;// Regular function on PE3-0
  GPIO_PORTE_DEN_R |= EPORT_0123;  // Enable digital on PE3-0
}