// Systick.c
// Test code for reflectance sensors. 
// Created by TYLOR FRANCA
// Modified by Min He
// 4/22/2025
// California State University, Long Beach

#ifndef SYSTICK_H
#define SYSTICK_H

#include <stdint.h>

void SysTick_Init(void);

void Wait_N_MS(uint32_t delay);

void Wait_N_US(uint32_t delay);

#endif