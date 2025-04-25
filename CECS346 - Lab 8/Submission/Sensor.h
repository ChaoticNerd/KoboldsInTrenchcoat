// Sensor.h
// Line Following Robot Starter program 
// Created by TYLOR FRANCA 
// February 2025
// CECS 497 SPRING 2025
// California State University, Long Beach
// Modified by Min He and Mark Joseph Austria
// 4/25/2025

#include <stdint.h>

// PE0 connect to sensor 7; PE1 connects to Sensor 0
// TODO: find the bit addresses
#define SENSORS           (*((volatile uint32_t *)0x4002400C))

#define BOTH_PWM                0xC0
#define LEFT_PWM				0x80
#define RIGHT_PWM				0x40
#define NO_PWM					0x00

void Sensor_Init(void);
uint8_t Sensor_CollectData(void);
