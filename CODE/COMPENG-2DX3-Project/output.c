#include "output.h"

// Set output LED indicators depending on the current action
void setOutput(const enum outputs outputType) {
	switch (outputType) {
		case NONE:
			for (int i = 0; i < 3; i++) setLEDs(i, 0);
			break;
		case M_START:
			setLEDs(0, 1);
			break;
		case M_END:
			setLEDs(0, 0);
			break;
		case U_START:
			setLEDs(1, 1);
			break;
		case U_END:
			setLEDs(1, 0);
			break;
		case W_START:
			setLEDs(2, 1);
			break;
		case W_END:
			setLEDs(2, 0);
			break;
	}
}

// Enable/disable physical LEDs
void setLEDs(const int LED, const int state) {
	switch (LED) {
		case 0:
			if (state) GPIO_PORTN_DATA_R |= 0x02;
			else GPIO_PORTN_DATA_R &= ~0x02;
			break;
		case 1:
			if (state) GPIO_PORTN_DATA_R |= 0x01;
			else GPIO_PORTN_DATA_R &= ~0x01;
			break;
		case 2:
			if (state) GPIO_PORTF_DATA_R |= 0x10;
			else GPIO_PORTF_DATA_R &= ~0x10;
			break;
	}
}