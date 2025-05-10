// LineFollower.c
// Line Following Robot Starter program 
// Created by TYLOR FRANCA 
// February 2025
// CECS 497 SPRING 2025
// California State University, Long Beach
// Modified by Min He and Mark Joseph Austria
// Finished by Natasha Kho, Justin Narciso, Hanna Estrada, William Grefaldeo
// 4/25/2025


#include "tm4c123gh6pm.h"
#include "Systick.h"
#include "Motors.h"
#include "Sensor.h"
#include "IR_Sensor.h"
#include <stdint.h>

#define NUM_STATES	17
/*#define DELAY_MOVING	5
#define DELAY_STOP	10*/
#define DELAY_SLOW_HIGH		10
#define DELAY_SLOW_LOW		90
#define DELAY_MED_HIGH		30
#define DELAY_MED_LOW			70
#define DELAY_FAST_HIGH		50
#define DELAY_FAST_LOW		50
#define STOP_DELAY				100

// Function prototypes
void System_Init(void);

struct State {
  uint8_t motors;              // controls motor power supply
  uint16_t delays;              // Delay in ms
  uint8_t next[NUM_STATES];   			// next state
};
typedef const struct State State_t;

// Center = move forward, left = turn left, right = turn right, stop = no movement
enum states {/*FORWARD_SLOW_HIGH, FORWARD_SLOW_LOW, FORWARD_MEDIUM_HIGH, FORWARD_MEDIUM_LOW,
						 LEFT_SLOW_HIGH		, LEFT_SLOW_LOW		, LEFT_MEDIUM_HIGH 	 , 	LEFT_MEDIUM_LOW,
						 RIGHT_SLOW_HIGH	,	RIGHT_SLOW_LOW	, RIGHT_MEDIUM_HIGH	 ,	RIGHT_MEDIUM_LOW,
						 RETRACK_LEFT_HIGH, RETRACK_LEFT_LOW, RETRACK_RIGHT_HIGH , 	RETRACK_RIGHT_LOW,
						 STOP_CAR*/
						FORWARD_SLOW_HIGH, FORWARD_SLOW_LOW,
						FORWARD_LEFT_TURN, LEFT_TURN_HIGH, LEFT_TURN_LOW,
						FORWARD_RIGHT_TURN, RIGHT_TURN_HIGH, RIGHT_TURN_LOW,
						STOP_CAR
						};
						

/*{Center,Left,Right,Stop}*/

State_t linefollower_fsm[NUM_STATES]={
 {FORWARD_HIGH, 50, {FORWARD_SLOW_LOW, FORWARD_SLOW_LOW, FORWARD_SLOW_LOW, FORWARD_SLOW_LOW}},
 {FORWARD_LOW,	450, {FORWARD_SLOW_HIGH, FORWARD_LEFT_TURN, FORWARD_RIGHT_TURN, STOP_CAR}},
 { FORWARD_HIGH,	75, {LEFT_TURN_HIGH, LEFT_TURN_HIGH, LEFT_TURN_HIGH, LEFT_TURN_HIGH}},
 {TURN_LEFT_HIGH,	75, {LEFT_TURN_LOW, LEFT_TURN_LOW, LEFT_TURN_LOW, LEFT_TURN_LOW}},
 {TURN_LEFT_LOW,	350, {FORWARD_SLOW_HIGH, FORWARD_LEFT_TURN, FORWARD_RIGHT_TURN, STOP_CAR}},
 {FORWARD_HIGH,	75, {RIGHT_TURN_HIGH, RIGHT_TURN_HIGH, RIGHT_TURN_HIGH, RIGHT_TURN_HIGH}},
 {TURN_RIGHT_HIGH ,	75, {RIGHT_TURN_LOW, RIGHT_TURN_LOW, RIGHT_TURN_LOW, RIGHT_TURN_LOW}},
 {TURN_RIGHT_LOW,	350, {FORWARD_SLOW_HIGH, FORWARD_LEFT_TURN, FORWARD_RIGHT_TURN, STOP_CAR}},
 {STOP,	500, {FORWARD_SLOW_HIGH, FORWARD_LEFT_TURN, FORWARD_RIGHT_TURN, STOP_CAR}},
	/*{FORWARD,			DELAY_MOVING,	{Center, Left, Right, Stop}},
	{TURN_LEFT,		DELAY_MOVING,	{Center, Left, Right, Stop}},
	{TURN_RIGHT, 	DELAY_MOVING,	{Center, Left, Right, Stop}},
	{STOP,				DELAY_STOP	,	{Center, Left, Right, Stop}},*/
	
		/*{FORWARD_HIGH			,	30	,		{FORWARD_SLOW_LOW, FORWARD_SLOW_LOW, FORWARD_SLOW_LOW, FORWARD_SLOW_LOW}},
		{FORWARD_LOW			,	70 	,		{FORWARD_SLOW_HIGH, LEFT_SLOW_HIGH, RIGHT_SLOW_HIGH, STOP_CAR}},
		{FORWARD_HIGH			,	10 	,		{FORWARD_MEDIUM_LOW, FORWARD_MEDIUM_LOW, FORWARD_MEDIUM_LOW, FORWARD_MEDIUM_LOW}},
		{FORWARD_LOW			,	90		,		{FORWARD_SLOW_HIGH, FORWARD_SLOW_HIGH, FORWARD_SLOW_HIGH, FORWARD_SLOW_HIGH}},
		{TURN_LEFT_HIGH		,	30	,		{LEFT_SLOW_LOW, LEFT_SLOW_LOW, LEFT_SLOW_LOW, LEFT_SLOW_LOW}},
		{TURN_LEFT_LOW		,	70	,		{FORWARD_SLOW_HIGH, FORWARD_MEDIUM_HIGH, RIGHT_SLOW_HIGH, RETRACK_LEFT_HIGH}},
		{TURN_LEFT_HIGH		,	DELAY_SLOW_HIGH	,		{LEFT_MEDIUM_LOW, LEFT_MEDIUM_LOW, LEFT_MEDIUM_LOW, LEFT_MEDIUM_LOW}},
		{TURN_LEFT_LOW		,	DELAY_SLOW_LOW		,		{LEFT_SLOW_HIGH, LEFT_SLOW_HIGH, LEFT_SLOW_HIGH, LEFT_SLOW_HIGH}},
		{TURN_RIGHT_HIGH 	,	30	,		{RIGHT_SLOW_LOW, RIGHT_SLOW_LOW, RIGHT_SLOW_LOW, RIGHT_SLOW_LOW}},
		{TURN_RIGHT_LOW  	,	70	,		{FORWARD_SLOW_HIGH, LEFT_SLOW_HIGH, FORWARD_MEDIUM_HIGH, RETRACK_RIGHT_HIGH}},
		{TURN_RIGHT_HIGH 	,	DELAY_SLOW_HIGH	,		{RIGHT_MEDIUM_LOW, RIGHT_MEDIUM_LOW, RIGHT_MEDIUM_LOW ,RIGHT_MEDIUM_LOW}},
		{TURN_RIGHT_LOW 	,	DELAY_SLOW_LOW		,		{RIGHT_SLOW_HIGH, RIGHT_SLOW_HIGH, RIGHT_SLOW_HIGH, RIGHT_SLOW_HIGH}},
		{TURN_LEFT_HIGH		,	70	,		{RETRACK_LEFT_LOW, RETRACK_LEFT_LOW, RETRACK_LEFT_LOW, RETRACK_LEFT_LOW}},
		{TURN_LEFT_LOW		,	30		,		{FORWARD_SLOW_HIGH, LEFT_SLOW_HIGH, RIGHT_SLOW_HIGH, STOP_CAR}},
		{TURN_RIGHT_HIGH 	,	70	,		{RETRACK_RIGHT_LOW, RETRACK_RIGHT_LOW, RETRACK_RIGHT_LOW, RETRACK_RIGHT_LOW}},
		{TURN_RIGHT_LOW 	,	30		,		{FORWARD_SLOW_HIGH, LEFT_SLOW_HIGH, RIGHT_SLOW_HIGH, STOP_CAR}},
		{STOP							,	STOP_DELAY			,		{FORWARD_SLOW_HIGH, LEFT_SLOW_HIGH, RIGHT_SLOW_HIGH, STOP_CAR}}*/
};

// new delays: DELAY_MED_HIGH, DELAY_MED_LOW, DELAY_SLOW_HIGH, DELAY_SLOW_LOW
// new states:
// FORWARD_SLOW_HIGH 		{FORWARD		,	DELAY_SLOW_HIGH	,		PWM_FORWARD_HIGH,	{FORWARD_SLOW_LOW, FORWARD_SLOW_LOW, FORWARD_SLOW_LOW, FORWARD_SLOW_LOW}},
// FORWARD_SLOW_LOW			{FORWARD		,	DELAY_SLOW_LOW 	,		PWM_FORWARD_LOW	,	{FORWARD_MEDIUM_HIGH, LEFT_SLOW_HIGH, RIGHT_SLOW_HIGH, STOP}},
// FORWARD_MEDIUM_HIGH	{FORWARD		,	DELAY_MED_HIGH 	,		PWM_FORWARD_HIGH,	{FORWARD_MEDIUM_LOW, FORWARD_MEDIUM_LOW, FORWARD_MEDIUM_LOW, FORWARD_MEDIUM_LOW}},
// FORWARD_MEDIUM_LOW		{FORWARD		,	DELAY_MED_LOW		,		PWM_FORWARD_LOW	,	{FORWARD_SLOW_HIGH, FORWARD_SLOW_HIGH, FORWARD_SLOW_HIGH, FORWARD_SLOW_HIGH}},
// LEFT_SLOW_HIGH				{TURN_LEFT	,	DELAY_SLOW_HIGH	,		PWM_LEFT_HIGH		,	{LEFT_SLOW_LOW, LEFT_SLOW_LOW, LEFT_SLOW_LOW, LEFT_SLOW_LOW}},
// LEFT_SLOW_LOW				{TURN_LEFT	,	DELAY_SLOW_LOW	,		PWM_LEFT_LOW		,	{FORWARD_SLOW_HIGH, LEFT_MEDIUM_HIGH, RIGHT_SLOW_HIGH, RETRACK_RIGHT_HIGH}},
// LEFT_MEDIUM_HIGH			{TURN_LEFT	,	DELAY_MED_HIGH	,		PWM_LEFT_HIGH		,	{LEFT_MEDIUM_LOW, LEFT_MEDIUM_LOW, LEFT_MEDIUM_LOW, LEFT_MEDIUM_LOW}},
// LEFT_MEDIUM_LOW			{TURN_LEFT	,	DELAY_MED_LOW		,		PWM_LEFT_LOW		,	{LEFT_SLOW_HIGH, LEFT_SLOW_HIGH, LEFT_SLOW_HIGH, LEFT_SLOW_HIGH}},
// RIGHT_SLOW_HIGH			{TURN_RIGHT,	DELAY_SLOW_HIGH	,		PWM_RIGHT_HIGH	,	{RIGHT_SLOW_LOW, RIGHT_SLOW_LOW, RIGHT_SLOW_LOW, RIGHT_SLOW_LOW}},
// RIGHT_SLOW_LOW				{TURN_RIGHT,	DELAY_SLOW_LOW	,		PWM_RIGHT_LOW		,	{FORWARD_SLOW_HIGH, LEFT_SLOW_HIGH, RIGHT_MEDIUM_HIGH, RETRACK_LEFT_HIGH}},
// RIGHT_MEDIUM_HIGH		{TURN_RIGHT,	DELAY_MED_HIGH	,		PWM_RIGHT_HIGH	,	{RIGHT_MEDIUM_LOW, RIGHT_MEDIUM_LOW, RIGHT_MEDIUM_LOW ,RIGHT_MEDIUM_LOW}},
// RIGHT_MEDIUM_LOW			{TURN_RIGHT,	DELAY_MED_LOW		,		PWM_RIGHT_LOW		,	{RIGHT_SLOW_HIGH, RIGHT_SLOW_HIGH, RIGHT_SLOW_HIGH, RIGHT_SLOW_HIGH}},
// RETRACK_LEFT_HIGH		{TURN_LEFT	,	DELAY_MED_HIGH	,		PWM_LEFT_HIGH		,	{RETRACK_LEFT_LOW, RETRACK_LEFT_LOW, RETRACK_LEFT_LOW, RETRACK_LEFT_LOW}},
// RETRACK_LEFT_LOW			{TURN_LEFT	,	DELAY_MED_LOW		,		PWM_LEFT_LOW		,	{FORWARD_SLOW_HIGH, LEFT_SLOW_HIGH, RIGHT_SLOW_HIGH, STOP}},
// RETRACK_RIGHT_HIGH		{TURN_RIGHT,	DELAY_MED_HIGH	,		PWM_RIGHT_HIGH	,	{RETRACK_RIGHT_LOW, RETRACK_RIGHT_LOW, RETRACK_RIGHT_LOW, RETRACK_RIGHT_LOW}},
// RETRACK_RIGHT_LOW		{TURN_RIGHT,	DELAY_MED_LOW		,		PWM_RIGHT_LOW		,	{FORWARD_SLOW_HIGH, LEFT_SLOW_HIGH, RIGHT_SLOW_HIGH, STOP}},
// STOP									{STOP			,	STOP_DELAY			,		PWM_STOP				,	{FORWARD_SLOW_HIGH, LEFT_SLOW_HIGH, RIGHT_SLOW_HIGH, STOP}}
// 

enum states curr_s;   // current state
uint8_t Input;	// define input var
bool move_car;

// update sensor data THEN start motor
// this way the systicks do not interfere
int main(void){
	// init system
	System_Init();
	
	// first state based on sensors' first input
	Input = Sensor_CollectData();
	curr_s = linefollower_fsm[curr_s].next[Input];;
	move_car = true;
		
	while (1) {
		// check if move car is true
		while (move_car) {
		MOTORS = linefollower_fsm[curr_s].motors;
			// wait some ms before updating sensor
		Wait_N_MS(linefollower_fsm[curr_s].delays);
			// update input from sensor, then update state
		Input = Sensor_CollectData();
		curr_s = linefollower_fsm[curr_s].next[Input];
		}
		MOTORS = STOP;
		while (!move_car) {
			WaitForInterrupt();
		};
		/*while (move_car) {
			NVIC_ST_CTRL_R &= ~NVIC_ST_CTRL_ENABLE;
			MOTORS = 0x00;
		}
		NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE;
		
		MOTORS = TURN_LEFT_HIGH;
		Wait_N_US(linefollower_fsm[LEFT_MEDIUM_HIGH].delays);*/
		// collect data even when ir sensor stops car
		/*MOTORS = STOP;
		Wait_N_US(linefollower_fsm[STOP_CAR].delays);*/
		/*Input = Sensor_CollectData();
		curr_s = linefollower_fsm[curr_s].next[Input];*/
		//MOTORS = linefollower_fsm[STOP].motors; //linefollower_fsm[STOP].motors;
		//while(!move_car) {
		//WaitForInterrupt();}
	}
}

void System_Init(void){
	DisableInterrupts();				// Disable interrupts in order to setup
	Motor_Init();
	Sensor_Init();
	IR_Sensor_Init();
	SysTick_Init();
	EnableInterrupts();
}

void GPIOPortD_Handler(void) {
	for(uint32_t i = 0; i < 320000; i ++){}	//Interrupt debounce
	move_car = !move_car;
	if(GPIO_PORTD_RIS_R & IR_SENSOR_MASK){			
		GPIO_PORTD_ICR_R = PD_ICR_VAL;				//resets interrupt value
		//Wait_N_US(linefollower_fsm[STOP_CAR].delays);
		
		
	}
	
}
