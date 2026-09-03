#include <stdint.h>
#include "tm4c1294ncpdt.h"
#include "PLL.h"
#include "SysTick.h"
#include "I2C0.h"
#include "VL53l1X_api.h"
#include "uart.h"

#define ToF_ADDRESS 0x29

// void getPortRegisters(char port, uint32_t* clock, uint32_t** DIR, uint32_t** DEN, uint32_t** alternateFunc, uint32_t** analogFunc);

// void dIO_Port_Init(char port, int dir, int pins);

void initAll();
// void PortM_Init();
// void PortF_Init();
// void PortN_Init();
// void PortH_Init();
// void EnableInt();
// void WaitForInt();