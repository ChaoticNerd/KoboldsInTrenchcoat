// CECS346 Project 1
// Team members: Justin Narciso, Natasha Kho, Hanna Estrada, William Grefaldeo
// Lab description: Creating two traffic lights and a pedestrian light using Moore finite state machine
 
// Hardware Design
// 1)	Port E will be used to control 4 LEDs: white(PE3), red (PE2), yellow (PE1), green (PE0).
// 2)	Port A will be used for the two switches: sw1 (PA2), sw2 (PA3)

#include <stdint.h>   // for data type alias

#include "SysTick.h"

// PB for car LEDS, PE for switches, PF for pedestrian LEDS (1 and 3)

// Registers for car LEDS
// Complete the following register definitions
#define T_LIGHT  							(*((volatile uint32_t *)0x400050FC)) // bit addresses for 6 LEDS (0-5) 
#define GPIO_PORTB_DIR_R        (*((volatile uint32_t *)0x40005400)) 
#define GPIO_PORTB_AFSEL_R      (*((volatile uint32_t *)0x40005420)) 
#define GPIO_PORTB_DEN_R        (*((volatile uint32_t *)0x4000551C)) 
#define GPIO_PORTB_AMSEL_R      (*((volatile uint32_t *)0x40005528)) 
#define GPIO_PORTB_PCTL_R       (*((volatile uint32_t *)0x4000552C)) 
	
// Registers for pedestrian LEDS
#define P_LIGHT  							(*((volatile uint32_t *)0x40025028)) // bit addresses for 2 LEDS (1 and 3) 
#define GPIO_PORTF_DIR_R        (*((volatile uint32_t *)0x40025400)) 
#define GPIO_PORTF_AFSEL_R      (*((volatile uint32_t *)0x40025420)) 
#define GPIO_PORTF_DEN_R        (*((volatile uint32_t *)0x4002551C)) 
#define GPIO_PORTF_AMSEL_R      (*((volatile uint32_t *)0x40025528)) 
#define GPIO_PORTF_PCTL_R       (*((volatile uint32_t *)0x4002552C)) 

//// Registers for switches
#define SENSORS                 (*((volatile uint32_t *)0x4002401C)) // bit addresses for the three switches (0-2)
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
#define BPORT_CTL        			0x00FFFFFF
#define EPORT_01       				0x03
#define EPORT_CTL        			0x000000FF
#define FPORT_13       				0x0A
#define FPORT_CTL        			0x0000F0F0
#define SYSCTL_RCGC2_GPIOB    	0x00000002    // port B Clock Gating Control
#define SYSCTL_RCGC2_GPIOE    	0x00000010    // port E Clock Gating Control
#define SYSCTL_RCGC2_GPIOF    	0x00000020    // port E Clock Gating Control
// 0.25 sec
#define QUARTER_SEC								1							// Quarter second delay (0.25 x 1)
#define ONE_SEC 								4							// One second delay (0.25 x 4)
#define TWO_SEC									8							// Two second delay (0.25 x 8)
#define NO_DELAY        						0             // Zero second delay (0.5 x 0)

#define GS_RW										100001 // (0x21)
#define YS_RW										010001 // (0x11)
#define RS_GW										001100 // (0x0C)
#define RS_YW										001010 // (0x0A)
#define RS_RW										001001 // (0x09)
// 1010 (bits 3, 1)									
#define	GP											1000   // (0x08)								
#define RP											0010   // (0x02)
#define RFP											0000   // (0x00)

#define NUM_STATES 							9             // number of states (using 9/16)
#define NUM_OUTS								2

// define each function 
void Delay(uint8_t n);
void Light_Init(void);
void Car_Light_Init(void);
void Sensor_Init(void);

// FSM state data structure
// Declares the data type for each element in array
struct Traffic_State {
	// components at [X][0] represent Out, [X][1] represent Duration of State, [X][2] represent Next State
  uint8_t Output[NUM_OUTS];
  uint8_t Time;  
  uint8_t Next[NUM_STATES];
}; 

typedef const struct State STyp;		// never changes


// Constants definitions
// states connected to each output
enum traffic_States {GoS, WaitS, GoW, WaitW, GoP, WaitPOn0, WaitPOff0, WaitPOn1, WaitPOff1};

// Output pins are: 3(white), 2(red), 1(yellow), 0(green)
// Input pins are: 1:sw2, 0:sw1 
// Declares and initializes values to Finite State Machine
// in order: next state for next input 000 001 010 011 100 101 110 111
// note: 100 101 110 111 are same states as 000 001 010 011 due to having same last 2 bits
// ** based on state diagram/table
STyp FSM[NUM_STATES]={
	{{GS_RW, RP},  TWO_SEC, 		{GoS, WaitS, WaitS, WaitS, GoS, WaitS, WaitS, WaitS}},											// GoS
	{{YS_RW, RP},  ONE_SEC, 		{GoW, GoP, GoW, GoW, GoP, GoP, GoW, GoW, GoP}},													// WaitS
	{{RS_GW, RP},  TWO_SEC, 		{GoW, WaitW, GoW, WaitW, WaitW, WaitW, WaitW, WaitW}},											// GoW
	{{RS_YW, RP},  ONE_SEC, 		{GoP, GoP, GoS, GoP, GoS, GoP, GoS, GoS}},														// WaitW
	{{RS_RW, GP},  TWO_SEC, 		{GoP, GoP, WaitPOn1, WaitPOn1, WaitPOn1, WaitPOn1, WaitPOn1, WaitPOn1}}							// GoP
	{{RS_RW, RFP}, QUARTER_SEC, 	{WaitPOff1, WaitPOff1, WaitPOff1, WaitPOff1, WaitPOff1, WaitPOff1, WaitPOff1, WaitPOff1}}		// WaitPOn1
	{{RS_RW, RP},  QUARTER_SEC,		{WaitPOn2, WaitPOn2, WaitPOn2, WaitPOn2, WaitPOn2, WaitPOn2, WaitPOn2, WaitPOn2}}				// WaitPOff1
	{{RS_RW, RFP}, QUARTER_SEC, 	{WaitPOff2, WaitPOff2, WaitPOff2, WaitPOff2, WaitPOff2, WaitPOff2, WaitPOff2, WaitPOff2}}		// WaitPOn2
	{{RS_RW, RP},  QUARTER_SEC,		{GoS, GoW, GoW, GoW, GoS, GoS, GoS, GoW}}														// WaitPOff2
};

int main(void){ 
  uint8_t S;  // index to the current state 
  uint8_t Input; 
	
	Light_Init();
	Car_Light_Init();
	Sensor_Init();
    S = GoS;                     // FSM start with green  
    
  while(1){
	// output LED color 
	T_LIGHT = FSM[S].Out[0];
	P_LIGHT = FSM[S].Out[1];
	// activate delay function with correct delay duration from current state
	Wait_N_Quart_Sec(FSM[S].Time);
	
	// Input = current button press value
	Input = SENSORS;
	// Update State
	S = FSM[S].Next[Input];
  }
}

// Port B Initialization
void Car_Light_Init(void){
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOB;       // Activate Port B clocks
	while ((SYSCTL_RCGC2_R & SYSCTL_RCGC2_GPIOB)!= SYSCTL_RCGC2_GPIOB) {} // wait for clock to be active
		
  GPIO_PORTB_AMSEL_R &= ~BPORT_012345;  // Disable analog function on PB0-5
  GPIO_PORTB_PCTL_R &= ~BPORT_CTL; // Enable regular GPIO
  GPIO_PORTB_DIR_R  |= BPORT_012345;  // Outputs on PB0-5
  GPIO_PORTB_AFSEL_R &= ~BPORT_012345; // Regular function on PB0-5
  GPIO_PORTB_DEN_R |= BPORT_012345;   // Enable digital signals on PB0-5
	// GPIO_PORTB_PDR_R |= BPORT_01234;  // Enable pull-down resistors for PB0-5 (Optional)
}

// Port E Initialization
void Ped_Light_Init(void){
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOE;      // Activate Port E clocks
	while ((SYSCTL_RCGC2_R & SYSCTL_RCGC2_GPIOE)!= SYSCTL_RCGC2_GPIOE) {} // wait for clock to be active
		
  GPIO_PORTE_AMSEL_R &= ~EPORT_01;// Disable analog function on PE0-1
  GPIO_PORTE_PCTL_R &= ~EPORT_CTL;// Enable regular GPIO
  GPIO_PORTE_DIR_R  |= EPORT_01;  // Outputs on PE0-1
  GPIO_PORTE_AFSEL_R &= ~EPORT_01;// Regular function on PE0-1
  GPIO_PORTE_DEN_R |= EPORT_01;  // Enable digital on PE0-1
}

// Port F Initialization
void Sensor_Init(void){
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF;      // Activate Port F clocks
	while ((SYSCTL_RCGC2_R & SYSCTL_RCGC2_GPIOF)!= SYSCTL_RCGC2_GPIOF) {} // wait for clock to be active
		
  GPIO_PORTF_AMSEL_R &= ~FPORT_13;// Disable analog function on PF1,3
  GPIO_PORTF_PCTL_R &= ~FPORT_CTL;// Enable regular GPIO
  GPIO_PORTF_DIR_R  &= ~FPORT_13;  // Inputs on PF1,3
  GPIO_PORTF_AFSEL_R &= ~FPORT_13;// Regular function on PF1,3
  GPIO_PORTF_DEN_R |= FPORT_13;  // Enable digital on PF1,3
}