// Motors.h
// Line Following Robot Starter program 
// Created by TYLOR FRANCA 
// February 2025
// CECS 497 SPRING 2025
// California State University, Long Beach
// Modified by Min He and Mark Joseph Austria
// 4/25/2025

#include "tm4c123gh6pm.h"
#include <stdint.h>

// Define motor outputs for different motions.
// PB7 for right motor and PB6 for left motor.
// PB5432 are the four direction pins:PB5:L:SLP,PB4:L:DIR, PB3:R:SLP,PB2:R:DIR
// SLP|DIR: 11: forward
// SLP|DIR: 10: backward
// TODO: find the constant values
// both on (connected to BOTH_PWM sensor)
#define FORWARD 		0x3C   // PWM(bits7,6):11, direction:0x3C
// connected to RIGHT_PWM sensor
#define TURN_LEFT 	0x01   // PWM(bits7,6):10, direction:0x3C
// connected to LEFT_PWM sensor
#define TURN_RIGHT 	0x02   // PWM(bits7,6):01, direction:0x3C
#define STOP   			0x00   // PWM(bits7,6):01, direction:0x3C
#define PERIOD 			160000 // PWM Period:10ms, value is based on 16MH system clock

// Global variables: 
// H: number of clocks cycles for duty cycle
// L: number of clock cycles for non-duty cycle
volatile uint32_t H,L;

static uint8_t pwm;  // two PWM signals on bits 7,6


// bit address definitions for port data registers
// PB67 for motor PWM signals: PB6 - Left DC Motor, PB7 - Right DC Motor
// PB5432 are the four direction pins:PB5:L:SLP,PB4:L:DIR, PB3:R:SLP,PB2:R:DIR
#define DIRECTION (*((volatile uint32_t *)0x400050F0)) 
#define MOTORS		(*((volatile uint32_t *)0x400053F0)) 

// This function initialize PB7-PB2 to output PWM signals and 
// direction signals for Two DC Motors:
// PB67 for motor PWM signals: PB6 - Left DC Motor, PB7 - Right DC Motor
// PB5432 for motor directions: PB54 - left DC Motor, PB32 - right DC Motor
void Motor_Init(void);

// This function will start motor in the direction specified by dir.
void Motor_Start(void);

// This function will stop motor movement.
void Motor_Stop(void);
	
