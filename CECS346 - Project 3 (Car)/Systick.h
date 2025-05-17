// Systick.h
// Line Following Robot Starter program 
// Created by TYLOR FRANCA 
// February 2025
// CECS 497 SPRING 2025
// California State University, Long Beach
// Modified by Min He and Mark Joseph Austria
// 4/25/2025

#include <stdint.h>

void SysTick_Init(void);

void Wait_N_MS(uint32_t delay);

void Wait_N_US(uint32_t delay);