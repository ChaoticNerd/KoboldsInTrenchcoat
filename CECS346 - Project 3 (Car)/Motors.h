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
// both on (connected to BOTH_PWM sensor)
#define FORWARD						0x03
#define TURN_LEFT					0x01
#define TURN_RIGHT				0x02
#define HALT							0x00


//static uint8_t pwm;  // two PWM signals on bits 7,6

// bit address definitions for port data registers
// PB67 for motor PWM signals: PB1 - Left DC Motor, PB0 - Right DC Motor
// PB5432 are the four direction pins:PB5:L:SLP,PB4:L:DIR, PB3:R:SLP,PB2:R:DIR
#define MOTORS (*((volatile uint32_t *)0x4000503C)) 

// This function initialize PB7-PB2 to output PWM signals and 
// direction signals for Two DC Motors:
// PB67 for motor PWM signals: PB6 - Left DC Motor, PB7 - Right DC Motor
// PB5432 for motor directions: PB54 - left DC Motor, PB32 - right DC Motor
void Motor_Init(void);

