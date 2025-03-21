// SysTick.h
// Runs on TM4C123
// Starter file for CECS346 Project 2
// By Dr. Min He


#include <stdint.h> // C99 data types

void SysTick_Init(void);

void SysTick_Start(uint32_t period);

void SysTick_Stop(void);