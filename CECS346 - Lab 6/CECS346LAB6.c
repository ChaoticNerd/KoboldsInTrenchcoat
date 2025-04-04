// Lab6_Interrupts.c
// Starter file for CECS346 Lab 6
// By Dr. Min He
// Edited by Natasha Kho, Justin Narciso, William Grefaldeo, Hanna Estrada
// 3/14/2024

#include "tm4c123gh6pm.h"
#include <stdint.h> // C99 data types

// 1. Pre-processor Directives Section
// Constant declarations to access port registers using 
// symbolic names instead of addresses

// TODO: define bit addresses for the three LEDs connected to PORTF
// need ports 0-3, SW2 as port0 and LED as port1-3
// HIGHEST PRI 0, LOWEST PRI 7
#define LED_ADR        0x40025038
#define SW2_ADR        0x40025004
#define PF_UNLOCK      0x4C4F434B
#define PORTF_INT_PRI  4U
#define PORTF_PRI_BITS 0x00800000
#define LED        (*((volatile uint32_t *)LED_ADR))   
#define SW2        (*((volatile uint32_t *)SW2_ADR))   
	
#define SW2_MASK        0x01 // Switch position check
#define RED  			      0x02 // Red LED
#define GREEN			      0x08 // Green LED
#define RGB             0x0E // Used to set PF1-3 to output
#define PF_3_0          0x0F // Controls PF0 (SW2)

// it's the usual 1/16000000 (16 million comes from 16MHz)
// For 0.5s, 16000000*0.5 = 8000000			// Assume the system clock is 16MHz; define number of clock cycles to generate 0.5s time interval.
#define HALF_S 							8000000U    // A U follows a constant indicate this is an unsigned int      
																				
#define PF3_0_GPIO 0x0000FFFF					
#define PF4_0_CNTRL 0x0F

// 0100 0000 0000 0000 0000 0000 0000 0000 = 0x40000000
#define NVIC_EN0_PORTF		  0x40000000  // bit position for PORTF interrupt in NVIC_EN0_R register.

#define EN_SYSTICK_CC				0x07 // Enable SysTick Core Clock and interrupts
#define PRI4_TOP3_BITS			0x1FFFFFFF // priority is 4, aka 0010 0000
#define PRI4_OTHER_BITS			0x20000000 // remaining bits become this

// Function Prototypes (external functions from startup.s)
extern void DisableInterrupts(void); // Disable interrupts
extern void EnableInterrupts(void);  // Enable interrupts
extern void WaitForInterrupt(void);  // Go to low power mode while waiting for the next interrupt

// Function Prototypes
// Initialize rising edge triggered interrupt for PF0 (SW2) and three LEDs on Port F
void Switch_LED_Init(void);  
void GPIOPortF_Handler(void);

// Initialize SysTick timer with interrupt enabled.
// Parameter "period" specifies number of counts for the time 
// period generated by systick timer.
void SysTick_Init(uint32_t period);  

// Defines SysTick_Handler funct
void SysTick_Handler(void);

// global variable visible in Watch and Memory window of debugger:
// used for the purpose of practicing debug: use watch window and memory window 
// to keep track of varaible values and memory contents.
// This variable helps keeping track of number of button presses: 
// it increments once per button release.
volatile uint32_t RisingEdges = 0;

// keep track of the current active LED 
volatile uint8_t curr_led = RED;

// Check to see if button has been pressed (which triggers the interrupt)
volatile uint8_t pressed = 0;

int main(void){
	DisableInterrupts();
  Switch_LED_Init();
	SysTick_Init(HALF_S);
	EnableInterrupts();
	
	// initialize current active LED to be red
	LED = RED;
	
  while(1){
		WaitForInterrupt();	

		// Want all if/method calls in Main, not ISR handler
		if(pressed){
			curr_led = (curr_led << 1);	// Turns on next LED
      if(curr_led&~RGB) {				// Checks to see if any of the LEDs are on
				curr_led = RED;						// Sets LED to RED since none of the LEDs are on (passed 0x08)
			}
			LED = curr_led;							// Sets Port value to what we want it to be
			pressed = 0;								// Exits ISR loop
    }
  }
}

// Implement Switch_LED_Init() to initialize the three onboard LEDs and rising edge
// triggered interrupt for PF0 (SW2) using friendly coding.
void Switch_LED_Init(void) {
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;     	// activate F clock
	while ((SYSCTL_RCGCGPIO_R&SYSCTL_RCGCGPIO_R5)!=SYSCTL_RCGCGPIO_R5){}; // wait for the clock to be ready
  
	// Normal Init Stuff
  GPIO_PORTF_LOCK_R = PF_UNLOCK; 			// unlock PF0
	GPIO_PORTF_CR_R |= PF4_0_CNTRL;     // allow changes to PF4-0 :11111->0x0F     
  GPIO_PORTF_DIR_R &= ~SW2_MASK;    	// (c) make PF0 in (built-in button), clear to make in
  GPIO_PORTF_DIR_R |= RGB; 						// make PF3-1 out (LEDs)
  GPIO_PORTF_AFSEL_R &= ~PF_3_0;  		// disable alt funct on PF3-0
  GPIO_PORTF_DEN_R |= PF_3_0;     		// enable digital I/O on PF3-0
  GPIO_PORTF_PCTL_R &= PF3_0_GPIO;  	// configure PF3-0 as GPIO
  GPIO_PORTF_AMSEL_R &= ~PF_3_0;      // disable analog functionality on PF
  GPIO_PORTF_PUR_R |= PF_3_0;     		// enable weak pull-up on PF4

  // Interrupt Init Stuff
  GPIO_PORTF_IS_R &= ~SW2_MASK;     	// (d) set PF0 to 0 to make it edge sensitive
  GPIO_PORTF_IBE_R &= ~SW2_MASK;    	//  set PF0 to 0 to state it's only sensitive to 1 edge
  GPIO_PORTF_IEV_R |= SW2_MASK;    		//  set PF0 to 1 to make it rising edge sensitive
  
  GPIO_PORTF_ICR_R |= SW2_MASK;     	// (e) clear flag0, note: writing 1 will clear bits in RIS
  // more on RIS: it flags at the port bit where the interrupt occurs, so
  // interrupting on PF0 flags RIS bit 0
  // 1 means an interrupt has occured
  // we are using ICR on bit 0 to clear that interrupt on initialization
  GPIO_PORTF_IM_R |= SW2_MASK;      	// (f) arm interrupt on PF0
  // sends interrupt signal to controller at corresponding bit

  // set PF0 to priority 4 interrupt
  // priority 4 = 100 0 0000 = 0x80
  // 0xFF1FFFFF is to clear bits 23-21 that determine PF priority
  NVIC_PRI7_R = (NVIC_PRI7_R&~PORTF_PRI_BITS)|PORTF_INT_PRI<<21; 	// (g) PORTF Interrupt priority bits: 23-21, priority set to 4

  NVIC_EN0_R |= NVIC_EN0_PORTF;    																// (h) PORTF interrupt number is 30, enable bit 30 in NVIC.     
}

// Parameter "period" specifies number of counts for the time 
void SysTick_Init(uint32_t period) {
  NVIC_ST_CTRL_R = 0; 							// clear to disable systick while setting up
  NVIC_ST_RELOAD_R = period - 1; 		// set reload to .5s
  NVIC_ST_CURRENT_R = 0; 						// clear current
																		
	
  NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R & PRI4_TOP3_BITS) | PRI4_OTHER_BITS ;		// SLIDE 27 LECTURE 6!!!
  NVIC_ST_CTRL_R = EN_SYSTICK_CC; 		// clk_src = 1, inten = 1, en = 1
} 

// Toggles through LED colors on port F (onboard LED)
void GPIOPortF_Handler(void) {
	// simple solution to take care of button debounce: 20ms to 30ms delay
  for (uint32_t i=0;i<160000;i++) {}
		
  // Round-robin LED Red --> Blue --> Green
  // RIS is Raw Interrupt Status
  if (GPIO_PORTF_RIS_R & SW2_MASK) {
		GPIO_PORTF_ICR_R |= SW2_MASK;
		pressed = 1;
    RisingEdges += 1;
  }
}

// When timer interrupt triggers, do what's necessary then toggle the current LED
void SysTick_Handler(void) {
  // Flash LED
  LED ^= curr_led; // XORed
}
