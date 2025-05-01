// Motor.c
// Line Following Robot Starter program 
// Created by TYLOR FRANCA 
// February 2025
// CECS 497 SPRING 2025
// California State University, Long Beach
// Modified by Min He and Mark Joseph Austria
// 4/25/2025

#include "tm4c123gh6pm.h"
#include <stdint.h>
#include "Motors.h"
#include "Sensor.h"
volatile uint32_t H, L;

// This function initialize PB7-PB2 to output PWM signals and 
// direction signals for Two DC Motors:
// defined in .h
// PB67 for motor PWM signals: PB6 - Left DC Motor, PB7 - Right DC Motor
// PB5432 for motor directions: PB54 - left DC Motor, PB32 - right DC Motor
void Motor_Init(uint32_t speed){
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOB;     	// activate B clock
	L = speed;
	H = PERIOD - H;
	
    GPIO_PORTB_AMSEL_R &= ~0x0F;  			// disable analog functionality 
    GPIO_PORTB_PCTL_R &= ~0xFFFFFFFF; 	// configure as GPIO
    GPIO_PORTB_DIR_R |= 0x0F;     			// set as output
    GPIO_PORTB_AFSEL_R &= ~0x0F;  			// disable alt funct 
    GPIO_PORTB_DEN_R |= 0x0F;     			// enable digital I/O 
}

// This function will start motor in the direction specified by dir.
void Motor_Start(void)
{
  NVIC_ST_RELOAD_R = H-1;       // reload value in number of clock cycles
  NVIC_ST_CURRENT_R = 0;        // any write to current clears it
	NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE;
}

void Motor_Stop(void)
{
	NVIC_ST_CTRL_R &= ~NVIC_ST_CTRL_ENABLE;
	GPIO_PORTE_DATA_R &= ~(RIGHT_PWM|LEFT_PWM); // output zero for PWM
}

// SysTick ISR:
// 1. Implement timing control for duty cycle and non-duty cycle
// 2. Output a waveform based on current duty cycle
void SysTick_Handler(void){
	
	// add LED
  if(MOTORS&pwm){   // previous cycle is duty cycle
    NVIC_ST_RELOAD_R = L-1;     // switch to non-duty cycle
  } else{ // previous cycle is non-duty cycle
    NVIC_ST_RELOAD_R = H-1;     // switch to duty cycle
  }
	
	MOTORS ^= pwm; // inverse output for PWM
}



