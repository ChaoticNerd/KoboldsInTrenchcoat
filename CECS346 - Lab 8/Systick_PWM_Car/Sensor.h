// Sensor.h
// Test code for reflectance sensors. 
// Created by TYLOR FRANCA
// Modified by Min He
// 4/22/2025
// California State University, Long Beach

#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>

#define SENSOR_CTRL       (*((volatile unsigned long *)0x4000500C))
#define SENSORS           (*((volatile unsigned long *)0x4002400C))
	
#define BOTH_PWM        0xC0
#define LEFT_PWM				0x80
#define RIGHT_PWM				0x40
#define NO_PWM					0x00

void Sensor_Init(void);
//uint8_t Sensor_CollectData(void);
// u-int bc it returns bit as an integer
uint8_t Sensor_CollectData(void);

#endif