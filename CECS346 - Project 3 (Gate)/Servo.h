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
#define SERVO_CENTER  24000    // 1.5ms duty cycle (at 16 MHz clock)
#define SERVO_CW_45   16000    // 1.0ms duty cycle (at 16 MHz clock)
#define SERVO_CCW_45  32000    // 2.0ms duty cycle (at 16 MHz clock)
#define SERVO_CW_90   8000     // 0.5ms duty cycle (at 16 MHz clock)
#define SERVO_CCW_90  40000    // 2.5ms duty cycle (at 16 MHz clock)
#define SERVO_PERIOD  32000   // 20ms period (at 16 MHz clock)

void SysTick_Init(void);

void Servo_Init(void);

void Drive_Servo(uint32_t angle);

#endif