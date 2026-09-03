#include <stdint.h>
#include "tm4c1294ncpdt.h"
#include "Motor.h"
#include "PLL.h"
#include "SysTick.h"
#include "input.h"

uint8_t place = 0;
uint16_t angle = 2048;

enum steps step = SMALL;
enum states state = STOP;
enum dirs direction = CW;

uint16_t Distance = 0;
uint16_t Signal = 0;
uint16_t Ambiant = 0;
uint16_t SpadNum = 0;
uint8_t RangeStatus = 255;

int main(void) {
	initAll(); // Initialize sensors and pins

	// FSM that controls overall function
	while (1) {
		if (state == STOP) { // No motion
			noSpin();
			SysTick_Wait(3600000);
			UART_printf("T\n\r");
			setOutput(NONE);
		} else if (direction == CW) { // Clockwise spin (taking measurements)
			cwSpin(&place); // Rotate motor 1 step clockwise
			angle--; // Update the current angle

			// Unwind wire if rotation is done
			if (angle == 0 || angle > 3000) { // Second case catches errors (angle is unsigned; becomes large and positive if it overflows)
				direction = CCW;
				VL53L1X_StopRanging(ToF_ADDRESS);
			}

			// Take measurement if at the right location
			if (angle%32) continue;

			setOutput(M_START); // Set measurement status indicator
			int status = getInput(&RangeStatus, &Distance, &Signal, &Ambiant, &SpadNum);
			setOutput(M_END); // Clear measurement status indicator

			// Mark the measurement as unvalid if collected data indicates it was unreliable
			int invalid = 0;
			if (!Distance) invalid = 1;
			if (Ambiant/SpadNum > 50) invalid = 1;
			if (RangeStatus > 2) invalid = 1;
			//if (Signal < 600) invalid = 1;
			setOutput(U_START); // Set UART status indicator
			//while (UART_InChar() != 's') {}
			sprintf(printf_buffer, "%u\n%u\n%u\n%u\n%u\n%u\n%u\n%u\n\r\n\r", Distance, angle, Signal, Ambiant, SpadNum, RangeStatus, status, invalid); // Prepare for transmission
			UART_printf(printf_buffer); // Transmit
			setOutput(U_END); // Clear UART status indicator
			// RangeStatus = 255; // Set back to error code to trigger next iteration
		} else if (direction == CCW) { // Counter-clockwise spin; unwinds wires after measurement collection
			setOutput(W_START);
			ccwSpin(&place); // Rotate motor 1 step counter-clockwise
			angle++; // Update the current angle

			// Stop and reset direction for next measurement
			if (angle >= 2048 && angle < 3000) { // Stopping the rotation, accounting for potential overflow
				state = STOP;
				direction = CW;
				setOutput(W_END);
			}
		}
	}
	return 0;
}

// Hardware interrupt handler
void GPIOJ_IRQHandler(void) {
	SysTick_Wait10ms(2); // Debounce
	if (GPIO_PORTJ_MIS_R & 0x1) { // Toggle between run and stop modes
		if (state == STOP) {
			state = RUN;
			// Begin measurements
			VL53L1X_StartRanging(ToF_ADDRESS);
		} else if (state == RUN) {
			state = STOP;
			VL53L1X_StopRanging(ToF_ADDRESS);
		}
		GPIO_PORTJ_ICR_R |= 0x1; // Clear event
	} if (GPIO_PORTJ_MIS_R & 0x2) { // Signal end of data collection
		if (direction == CCW || state == STOP) {
			setOutput(U_START); // Set status indicator
			UART_printf("T\n\r\n\r"); // Transmit
			setOutput(U_END); // Clear status indicator
			//setOutput(W_START); // Clear status indicator
			//home(&place, &angle);
			//setOutput(W_END); // Clear status indicator
		}
		GPIO_PORTJ_ICR_R |= 0x2;
	}
}