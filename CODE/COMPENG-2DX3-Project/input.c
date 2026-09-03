#include "input.h"

void debounce() {
	SysTick_Wait(36000); // Wait 1ms
}

int getInput(uint8_t* RangeStatus, uint16_t* Distance, uint16_t* Signal, uint16_t* Ambiant, uint16_t* SpadNum) {
	uint8_t dataReady = 0;
	uint16_t maxSig = 0;
	uint16_t bestDist = 0;
	uint16_t bestAmb = 0;
	uint16_t bestSpad = 0;
	uint16_t bestStatus = 0;
	*RangeStatus = 255;
	int senStatus = 0;
	int status = 0;

	// Retry until a valid reading is received (RangeStatus and status are 0) or 10 attempts have been made
	for (int i = 0; i < 10 && (*RangeStatus || status); i++) {
		status = 0;
		while (!dataReady) {
			status |= VL53L1X_CheckForDataReady(ToF_ADDRESS, &dataReady);
			status |= VL53L1_WaitMs(ToF_ADDRESS, 5);
		}
		dataReady=0;

		// Check distance and supporting measurements
		status |= VL53L1X_GetDistance(ToF_ADDRESS, Distance);
		status |= VL53L1X_GetSignalRate(ToF_ADDRESS, Signal);
		status |= VL53L1X_GetAmbientRate(ToF_ADDRESS, Ambiant);
		status |= VL53L1X_GetSpadNb(ToF_ADDRESS, SpadNum);
		status |= VL53L1X_GetRangeStatus(ToF_ADDRESS, RangeStatus);
		if (status) i--;
		if (*Signal > maxSig) { // Keep the reading with the strongest signal for best results
			bestDist = *Distance;
			maxSig = *Signal;
			bestAmb = *Ambiant;
			bestStatus = *RangeStatus;
			senStatus = status;
			bestSpad = *SpadNum;
		}
	}

	// Pass results back through pointers
	*Distance = bestDist;
	*Signal = maxSig;
	*Ambiant = bestAmb;
	*RangeStatus = bestStatus;
	*SpadNum = bestSpad;
	return senStatus;
}