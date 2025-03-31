// Sensors_Lights.c
// Starter file for CECS346 Project 2
// By Dr. Min He
// Edited by Natasha Kho, Justin Narciso, Hanna Estrada, William Grefaldeo
#include "tm4c123gh6pm.h"
#include <stdint.h> // C99 data types
#include "Sensors_Lights.h"

#define SWITCHES_MASK   0x0C  // masks PA3-2
#define RESET_MASK      0x04  // masks PE2
#define LIGHTS_MASK     0xFF  // masks PB7-0
#define NVIC_EN0_PORTA  0x00000001  // For PA2, PA3; matches IRQ for PORTA_HANDLER
#define NVIC_EN0_PORTE  0x00000010  // Matches IRQ for PORTE_HANDLER
#define GPIO_PORTCTRL_SENSORS   0x0000FF00  // targets PA3-2 port control
#define GPIO_PORTCTRL_RESET			0x00000F00  // targets PE2 port control
#define PORTA_PRI_BITS  0x20        // Clears bits 7-5
#define PORTE_PRI_BITS  0x10        // Clears bits 7-5
#define PORTB_CNTRL    	0xFFFFFFFF  // targets PB7-0 port control
#define PORTA_INT_PRI   2U          // Priority 2 for both edge trigger
#define PORTE_INT_PRI   1U          // Priority 1


// Initialize the two sensors, enable both edge edge-triggered interrupt for both sensors
void Sensors_Init(void){ // matches Switch_LED_Init
    // PORT A 2,3
		// Using legacy mode; next time use SYSCTL_RCGCGPIO
    SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA;                                // Set up PORT A Clock
    while ((SYSCTL_RCGC2_R & SYSCTL_RCGC2_GPIOA)!= SYSCTL_RCGC2_GPIOA) {}  // Waits until clock is ready

    // Port A Init Stuff
    GPIO_PORTA_DIR_R &= ~SWITCHES_MASK;     // Sets port to input
    GPIO_PORTA_AFSEL_R &= ~SWITCHES_MASK;   // Disables all alt functions on port A2, A3
    GPIO_PORTA_DEN_R |= SWITCHES_MASK;      // Enables digital I/O on A2, A3
    GPIO_PORTA_PCTL_R &= ~GPIO_PORTCTRL_SENSORS;      // Sets PA2, PA3 to GPIO
    GPIO_PORTA_AMSEL_R &= ~SWITCHES_MASK;   // Disables analogue functionality of PA2, PA3
    GPIO_PORTA_PUR_R &= ~SWITCHES_MASK;      // Disables pull-up, thus using pull-down, thus hardware is positive

    // Interrupt Init Stuff; need 01X1
    GPIO_PORTA_IS_R  &= ~SWITCHES_MASK;     // Enables Edge sensitive
    GPIO_PORTA_IBE_R |=  SWITCHES_MASK;     // Enables Both-edge sensitive
    // Dont need IEV
    GPIO_PORTA_IM_R  |=  SWITCHES_MASK;     // Enables interrupt on A2, A3

    // Interrupt Controller
    NVIC_PRI0_R = (NVIC_PRI0_R&~PORTA_PRI_BITS)|PORTA_INT_PRI<<5; // Priority bits are 7-5, so shift 5
    NVIC_EN0_R |= NVIC_EN0_PORTA;																	// BIT NEEDS TO MATCH IRQ
}

// Initialize the reset button: use level triggered interrupt
void Reset_Init(void){
		// set up port e clock and wait for clock to be ready
    SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOE;
    while((SYSCTL_RCGC2_R & SYSCTL_RCGC2_GPIOE) != SYSCTL_RCGC2_GPIOE);

    //Port E Init Stuff
    GPIO_PORTE_DIR_R &= ~RESET_MASK;     // sets port to input
    GPIO_PORTE_AFSEL_R &= ~RESET_MASK;   //  disables all alt function on PE2 
    GPIO_PORTE_DEN_R |= RESET_MASK;      //  Enables digital I/O on PE2
    GPIO_PORTE_PCTL_R &= ~GPIO_PORTCTRL_RESET;    // PE2 sets to GPIO 
    GPIO_PORTE_AMSEL_R &= ~RESET_MASK;   // Disables analog funtion PE2
    GPIO_PORTE_PUR_R &= ~RESET_MASK;      // Weak pull down on PE2
    
    //Interrupt Init Stuff; need 1X11
    GPIO_PORTE_IS_R  |= RESET_MASK;         // Enables Level sensitive
    GPIO_PORTE_IBE_R &= ~RESET_MASK;        // check interrupt event iev
    GPIO_PORTE_IEV_R |= RESET_MASK;			// choose high level sensitive = 1
    GPIO_PORTE_IM_R  |= RESET_MASK;         // Enable interrupt

    // Interrupt Controller
    NVIC_PRI1_R = (NVIC_PRI1_R&~PORTE_PRI_BITS)|PORTE_INT_PRI<<5; // clear priority bits and set priority to 1
    NVIC_EN0_R |= NVIC_EN0_PORTE;																	// BIT NEEDS TO MATCH IRQ
}

// Initialize 8 LEDs
void Lights_Init(void){ // PORT B0 - B7
    SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOB;       // Activate Port B clocks
		while ((SYSCTL_RCGC2_R & SYSCTL_RCGC2_GPIOB)!= SYSCTL_RCGC2_GPIOB) {} // wait for clock to be active
	
    GPIO_PORTB_AMSEL_R &= ~LIGHTS_MASK;     // Disable analog function on PB0-5
    GPIO_PORTB_PCTL_R &= ~PORTB_CNTRL; 		// Enable regular GPIO
    GPIO_PORTB_DIR_R  |= LIGHTS_MASK;  	    // Outputs on PB0-7
    GPIO_PORTB_AFSEL_R &= ~LIGHTS_MASK; 	// Regular function on PB0-7
    GPIO_PORTB_DEN_R |= LIGHTS_MASK;   	    // Enable digital signals on PB0-7
}
