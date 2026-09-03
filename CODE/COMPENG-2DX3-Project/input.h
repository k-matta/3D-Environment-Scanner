#include <stdint.h>
#include "tm4c1294ncpdt.h"
#include "setup.h"

void debounce();
int getInput(uint8_t* RangeStatus, uint16_t* Distance, uint16_t* Signal, uint16_t* Ambiant, uint16_t* SpadNum);