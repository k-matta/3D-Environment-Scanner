// COMPENG 2DX3
// This program illustrates the interfacing of the Stepper Motor with the microcontroller

//  Written by Ama Simons
//  January 18, 2020
// 	Last Update by Dr. Shahrukh Athar on February 2, 2025

#include "Motor.h"

void noSpin() {
	GPIO_PORTH_DATA_R = 0b00000000;
}

// Spin the motor counter-clockwise by one step
void ccwSpin(uint8_t *place){
	switch (*place) {
		case 0:
			GPIO_PORTH_DATA_R = 0b00000011;
			SysTick_Wait10us(DELAY);
			(*place)++;
			break;
		case 1:
			GPIO_PORTH_DATA_R = 0b00000110;
			SysTick_Wait10us(DELAY);
			(*place)++;
			break;
		case 2:
			GPIO_PORTH_DATA_R = 0b00001100;
			SysTick_Wait10us(DELAY);
			(*place)++;
			break;
		case 3:
			GPIO_PORTH_DATA_R = 0b00001001;
			SysTick_Wait10us(DELAY);
			(*place) = 0;
			break;
	}
}

// Spin the motor clockwise by one step
void cwSpin(uint8_t *place) {
	switch (*place) {
		case 0:
			GPIO_PORTH_DATA_R = 0b00000011;
			SysTick_Wait10us(DELAY);
			*place = 3;
			break;
		case 1:
			GPIO_PORTH_DATA_R = 0b00000110;
			SysTick_Wait10us(DELAY);
			(*place)--;
			break;
		case 2:
			GPIO_PORTH_DATA_R = 0b00001100;
			SysTick_Wait10us(DELAY);
			(*place)--;
			break;
		case 3:
			GPIO_PORTH_DATA_R = 0b00001001;
			SysTick_Wait10us(DELAY);
			(*place)--;
			break;
	}
}

// Move sensor to HOME position
void home(uint8_t* place, uint16_t* angle) {
	while (*angle < 2048) {
		ccwSpin(place);
		(*angle)++;
	}
}