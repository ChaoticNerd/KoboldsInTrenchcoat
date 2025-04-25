// Measure_Sensor_Decay.c
// Sensor Decay Test Code 
// Created by TYLOR FRANCA 
// February 2025
// CECS 497 SPRING 2025
// California State University, Long Beach
// Modified by Min He


#include "tm4c123gh6pm.h"
#include "Systick.h"
#include "Sensor.h"

int main(void){
	// init both
	Sensor_Init();
	SysTick_Init();

  while(1){
		// literally just collecting sensor data
		// in short:
		// constnatly charges caps, waits, then turns on input, collects data, turns off, then waits and restarts
		CollectSensorData(); 
  }
}


