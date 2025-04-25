// SensorTestWithLEDs.c.c
// This progrram is provide code to test reflectance sensors. 
// Created by Min He
// 4/22/2025
// California State University, Long Beach



#include "tm4c123gh6pm.h"
#include "Systick.h"
#include "Sensor.h"
#include "LED.h"

int main(void){
  Sensor_Init();  
	LED_Init();

  while(1){
		switch (Sensor_CollectData()) {
			case 0: 
				LED = RED;  // center
			  break;
			case 1: 			// right
				LED = GREEN;
			  break;
			case 2:				// left
				LED = BLUE;
			  break;
			case 3: 			// lost
				LED = WHITE;
			  break;
			default:
				LED = DARK;
			  break;
		}			
  }
}


