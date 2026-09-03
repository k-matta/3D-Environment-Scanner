#include <stdint.h>
#include "tm4c1294ncpdt.h"
#include "global.h"

enum outputs {NONE, M_START, M_END, U_START, U_END, W_START, W_END};

void setLEDs(int LED, int state);

// void setOutput(enum states currentState, enum dirs currentDir, enum steps currentStep, int angle);
void setOutput(enum outputs outputType);