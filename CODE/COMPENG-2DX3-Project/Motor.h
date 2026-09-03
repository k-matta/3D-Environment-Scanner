#include <stdint.h>
#include "tm4c1294ncpdt.h"
#include "SysTick.h"
#include "output.h"

#define DELAY 500
#define FULL_ROTATION 2048

void noSpin();
void cwSpin(uint8_t *place);
void ccwSpin(uint8_t *place);
void home(uint8_t *place, uint16_t *angle);