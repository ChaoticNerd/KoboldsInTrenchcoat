// Sensor.h
// Sensor Decay Test Code 
// Created by TYLOR FRANCA 
// February 2025
// CECS 497 SPRING 2025
// California State University, Long Beach
// Modified by Min He

#include <stdint.h>

// PORTE pin assignments
// PE0: Sensor 0  
// PE1: CTRL even  
// PE2: Sensor 0 read back digital value 
#define SENSOR0       				(*((volatile uint32_t *)0x40024004))  // PE0
#define CTRL_EVEN       			(*((volatile uint32_t *)0x40024008))  // PE1
#define SENSOR0_DIGITAL       (*((volatile uint32_t *)0x40024010))  // PE2

#define SENSOR0_MASK 					0x01
#define CONTRL_EVEN_MASK 			0x02
#define SENSOR0_DIGITAL_MASK 	0x04

#define SENSOR0_DIR_R GPIO_PORTE_DIR_R

void Sensor_Init(void);
void CollectSensorData(void);
