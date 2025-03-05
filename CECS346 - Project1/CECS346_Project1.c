// CECS346 Project 1
// Team members: Justin Narciso, Natasha Kho, Hanna Estrada, William Grefaldeo
// Lab description: Creating a traffic light using Moore finite state machine

// Hardware Design
// 1)	Port E will be used to control 4 LEDs: white(PE3), red (PE2), yellow (PE1), green (PE0).
// 2)	Port A will be used for the two switches: sw1 (PA2), sw2 (PA3)

#include <stdint.h>   // for data type alias

#include "SysTick.h"

// PB for car LEDS, PE for switches, PF for pedestrian LEDS (1 and 3)

// Registers for car LEDS
// Complete the following register definitions
#define CAR_LIGHT  							(*((volatile uint32_t *)0x400050FC)) // bit addresses for 6 LEDS (0-5) 
#define GPIO_PORTB_DIR_R        (*((volatile uint32_t *)0x40005400)) 
#define GPIO_PORTB_AFSEL_R      (*((volatile uint32_t *)0x40005420)) 
#define GPIO_PORTB_DEN_R        (*((volatile uint32_t *)0x4000551C)) 
#define GPIO_PORTB_AMSEL_R      (*((volatile uint32_t *)0x40005528)) 
#define GPIO_PORTB_PCTL_R       (*((volatile uint32_t *)0x4000552C)) 
	
// Registers for pedestrian LEDS
#define PED_LIGHT  							(*((volatile uint32_t *)0x40025028)) // bit addresses for 2 LEDS (1 and 3) 
#define GPIO_PORTF_DIR_R        (*((volatile uint32_t *)0x40025400)) 
#define GPIO_PORTF_AFSEL_R      (*((volatile uint32_t *)0x40025420)) 
#define GPIO_PORTF_DEN_R        (*((volatile uint32_t *)0x4002551C)) 
#define GPIO_PORTF_AMSEL_R      (*((volatile uint32_t *)0x40025528)) 
#define GPIO_PORTF_PCTL_R       (*((volatile uint32_t *)0x4002552C)) 

//// Registers for switches
#define SENSOR                 	(*((volatile uint32_t *)0x4002401C)) // bit addresses for the three switches (0-2)
#define GPIO_PORTE_DATA_R       (*((volatile uint32_t *)0x400243FC)) // calls all pins in port to be able to read/write
#define GPIO_PORTE_DIR_R        (*((volatile uint32_t *)0x40024400)) 
#define GPIO_PORTE_AFSEL_R      (*((volatile uint32_t *)0x40024420)) 
#define GPIO_PORTE_DEN_R        (*((volatile uint32_t *)0x4002451C)) 
#define GPIO_PORTE_AMSEL_R      (*((volatile uint32_t *)0x40024528))
#define GPIO_PORTE_PCTL_R       (*((volatile uint32_t *)0x4002452C))
#define GPIO_PORTE_PDR_R        (*((volatile uint32_t *)0x40024514)) // for resistor
#define SYSCTL_RCGC2_R          (*((volatile uint32_t *)0x400FE108)) // activate internal clock
	
// Constants definitions
#define BPORT_012345         		0x3F
#define BPORT_CTL        				0x00FFFFFF
#define EPORT_012       				0x07
#define EPORT_CTL        				0x00000FFF
#define SYSCTL_RCGC2_GPIOB    	0x00000002    // port B Clock Gating Control
#define SYSCTL_RCGC2_GPIOE    	0x00000010    // port E Clock Gating Control
#define HALF_SEC 								1   					// Half second delay (0.5 x 1)
#define ONE_SEC 								2							// One second delay (0.5 x 2)
#define TWO_SEC									4							// Two second delay (0.5 x 4)
#define NO_DELAY        				0             // Zero second delay (0.5 x 0)

/*#define LED_GREEN       				0x01          // green LED (port E0)
#define LED_YELLOW 							0x02          // yellow LED (port E1)
#define LED_REDWHITE_0 					0x0C          // red and white LED (port E2, E3)
#define LED_RED 								0x04          // red LED (port E0)
#define LED_REDWHITE_1 					0x0C          // red and white LED (port E2, E3)
#define SENSOR_SHIFT 						2             // shift SENSOR to right by 2 (since we're using port A2 and A3)*/

#define S_GREEN
#define S_YELLOW
#define S_RED
#define W_GREEN
#define W_YELLOW
#define W_RED
#define PED_

#define NUM_STATES 							9             // number of states (using 9/16)

// define each function
void Delay(uint8_t n);
void Light_Init(void);
void Sensor_Init(void);

// FSM state data structure
// Declares the data type for each element in array
struct Traffic_State {
	// components at [X][0] represent Out, [X][1] represent Duration of State, [X][2] represent Next State
  uint8_t Out; 
  uint8_t Time;  
  uint8_t Next[NUM_STATES];
}; 

typedef const struct State STyp;		// never changes


// Constants definitions
// states connected to each output
enum Traffic_States {GoS, WaitS, GoW, WaitW, GoP, WaitPOn0, WaitPOff0, WaitPOn1, WaitPOff1};

// Output pins are: 3(white), 2(red), 1(yellow), 0(green)
// Input pins are: 1:sw2, 0:sw1 
// Declares and initializes values to Finite State Machine
// in order: next state for next input 000 001 010 011 100 101 110 111
// note: 100 101 110 111 are same states as 000 001 010 011 due to having same last 2 bits
// ** based on state diagram/table
STyp FSM[NUM_STATES]={
	{ 
	
	
	
	
	
	
	
	
	}
};



int main(void){ 
  uint8_t S;  // index to the current state 
  uint8_t Input; 
	
	Light_Init();
	Sensor_Init();
  S = GREEN;                     // FSM start with green  
    
  while(1){
		/*// output LED color 
		LIGHT = FSM[S].Out;
		// activate delay function with correct delay duration from current state
		// Delay(FSM[S].Time);
		Wait_N_Half_Sec(FSM[S].Time);
		
		// 00001100
		// --> 11110011 (when pressed)
		// Input = Current Button press value, AND SENSOR with APORT_23 to clear other bits
		Input = (((~SENSOR & APORT_23) >> SENSOR_SHIFT));
		// Update State
		S = FSM[S].Next[Input];*/
  }
}

// delay function
/*void Delay(uint8_t n_500ms){
	volatile uint32_t time;
	
  time = n_500ms*(727240*100/91);  // 0.5sec
  while(time){
		time--;
  }
}*/

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