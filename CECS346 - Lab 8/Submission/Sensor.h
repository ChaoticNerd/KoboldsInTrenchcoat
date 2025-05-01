// Sensor.h
// Line Following Robot Starter program 
// Created by TYLOR FRANCA 
// February 2025
// CECS 497 SPRING 2025
// California State University, Long Beach
// Modified by Min He and Mark Joseph Austria
// 4/25/2025

#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>

// GPIO data register bit address definitions
// Reflectance sensor pin connections:
// PE3: Even CTRL, PE2: ODD CTRL
// PE1: Left Sensor, PE0: Right Sensor
#define SENSOR_CTRL       (*((volatile unsigned long *)0x40024030)) // Port E 3,2
#define SENSORS           (*((volatile uint32_t *)0x4002400C)) // Port E 1,0
// TODO: connect to PB and PE data

// both wheels active (sensor both white)
#define BOTH_PWM        0xC0
// left wheel active (turn right) (sensor right white)
#define LEFT_PWM				0x80
// right wheel active (turn left) (sensor left white)
#define RIGHT_PWM				0x40
// stop movement (sensors both black)
#define NO_PWM					0x00

void Sensor_Init(void);
uint8_t Sensor_CollectData(void);

#endif