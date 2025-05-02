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

// This function initialize PB7-PB2 to output PWM signals and 
// direction signals for Two DC Motors:
// motor and direction are defined in .h
#define PORTB_BITS	0x0F // bits 3-0
#define PORTB_PTCL	0x0000FFFF

// PB67 for motor PWM signals: PB6 - Left DC Motor, PB7 - Right DC Motor
// PB5432 for motor directions: PB54 - left DC Motor, PB32 - right DC Motor
void Motor_Init(){
		SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;     	// activate B clock
	
    GPIO_PORTB_AMSEL_R &= ~PORTB_BITS;  			// disable analog functionality 
    GPIO_PORTB_PCTL_R &= ~PORTB_PTCL; 	// configure as GPIO
    GPIO_PORTB_DIR_R |= PORTB_BITS;     			// set as output
    GPIO_PORTB_AFSEL_R &= ~PORTB_BITS;  			// disable alt funct 
    GPIO_PORTB_DEN_R |= PORTB_BITS;     			// enable digital I/O 
}