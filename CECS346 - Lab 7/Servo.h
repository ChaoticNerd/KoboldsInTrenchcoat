// Servo.h
// Example program to test servo motor.
// This file provide driver function prototypes and publich constant definitions.
// By Min He
// 4/17/2025
#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>

// PWM duty cycle systick timer reload values for 0.5ms - 2.5ms duty cycle
// period is 20ms
#define SERVO_START   24000    // 1.5ms duty cycle (at 16 MHz clock), 90 degrees
#define SERVO_END	  48000    // 3.0ms duty cycle (at 16 MHz clock)
#define SERVO_PERIOD  320000   // 20ms period (at 16 MHz clock)

void SysTick_Init(void);

void Servo_Init(void);

void Drive_Servo(uint32_t angle);

void Stop_Servo(void);

#endif