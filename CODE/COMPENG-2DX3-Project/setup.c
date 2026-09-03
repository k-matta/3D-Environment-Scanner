#include "setup.h"

// Enable interrupts
void EnableInt(void) {
	__asm("    cpsie   i\n");
}

//void WaitForInt(void) {
//	__asm("    wfi\n");
//}

// Set up port N with the required settings
void PortN_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;			// Activate the clock for Port N
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R12) == 0){};	// Allow time for clock to stabilize
	
	GPIO_PORTN_DIR_R=0b00000011;						// Enable PN0 and PN1 as outputs													
	GPIO_PORTN_DEN_R=0b00000011;						// Enable PN0 and PN1 as digital pins
	return;
}

// Set up port M with the required settings
void PortM_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;			// Activate the clock for Port N
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){};	// Allow time for clock to stabilize
	
	GPIO_PORTM_DIR_R=0b00000001;						// Enable PN0 and PN1 as outputs													
	GPIO_PORTM_DEN_R=0b00000001;						// Enable PN0 and PN1 as digital pins
	return;
}

// Enable LED D3, D4. Remember D3 is connected to PF4 and D4 is connected to PF0
void PortF_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;				// Activate the clock for Port F
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R5) == 0){};	// Allow time for clock to stabilize
	
	GPIO_PORTF_DIR_R=0b00010000;						// Enable PF0 and PF4 as outputs
	GPIO_PORTF_DEN_R=0b00010000;						// Enable PF0 and PF4 as digital pins
	return;
}

// Set up port H with the required settings
void PortH_Init() {
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;			// activate clock for Port H
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){};	// allow time for clock to stabilize
	GPIO_PORTH_DIR_R |= 0x0F;        					// configure Port M pins (PH0-PH3) as output
	GPIO_PORTH_AFSEL_R &= ~0x0F;     					// disable alt funct on Port H pins (PH0-PH3)
	GPIO_PORTH_DEN_R |= 0x0F;        					// enable digital I/O on Port H pins (PH0-PH3)
	// configure Port H as GPIO
	GPIO_PORTH_AMSEL_R &= ~0x0F;     					// disable analog functionality on Port H pins (PH0-PH3)	
	return;
}

// Set up port J with the required settings
void PortJ_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;			// Activate clock for Port J
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R8) == 0){};	// Allow time for clock to stabilize
	GPIO_PORTJ_DIR_R &= ~0x03;    						// Make PJ1 input 
	GPIO_PORTJ_DEN_R |= 0x03;     						// Enable digital I/O on PJ0-PJ1
	
	GPIO_PORTJ_PCTL_R &= ~0x000000F0;					// Configure PJ0-PJ1 as GPIO 
	GPIO_PORTJ_AMSEL_R &= ~0x03;						// Disable analog functionality on PJ0-PJ1		
	GPIO_PORTJ_PUR_R |= 0x03;							// Enable weak pull up resistor on PJ0-PJ1
}

// Setup the time of flight sensor with the required settings
void ToF_Init(void) {
	int status;
	// UART_printf("Starting sensor bootup...\n\r");
	uint8_t sensorState = 0;
	while (!sensorState) {
		status = VL53L1X_BootState(ToF_ADDRESS, &sensorState);
		SysTick_Wait10ms(10);
	}
	status = VL53L1X_ClearInterrupt(ToF_ADDRESS);
	status = VL53L1X_SensorInit(ToF_ADDRESS);
	// Status_Check("SensorInit", status);
	status = VL53L1X_StopRanging(ToF_ADDRESS);

	// Setting important values
	//status = VL53L1X_SetDistanceMode(ToF_ADDRESS, 2);
	//sprintf(printf_buffer, "Set Distance Mode to Long: %u\n\r", status);
	//UART_printf(printf_buffer);

	/*status = VL53L1X_SetTimingBudgetInMs(ToF_ADDRESS, 200);
	sprintf(printf_buffer, "Set TB to 200ms: %u\n\r", status);
	UART_printf(printf_buffer);
	
	uint16_t data;
	
	VL53L1X_GetTimingBudgetInMs(ToF_ADDRESS, &data);
	sprintf(printf_buffer, "Timing Budget: %u\n\r", data);
	UART_printf(printf_buffer);
	
	status = VL53L1X_SetSignalThreshold(ToF_ADDRESS, 512);
	sprintf(printf_buffer, "Set Signal Threshold to 800: %u\n\r", status);
	UART_printf(printf_buffer);

	// Verifying setup
	//VL53L1X_GetDistanceMode(ToF_ADDRESS, &data);
	//sprintf(printf_buffer, "Distance Mode: %u\n\r", data);
	//UART_printf(printf_buffer);
	
	
	VL53L1X_GetSignalThreshold(ToF_ADDRESS, &data);
	sprintf(printf_buffer, "Signal Limit: %u\n\r", data);
	UART_printf(printf_buffer);

	//status = VL53L1X_SetROI(ToF_ADDRESS, 8, 8);
	uint16_t* Y;
	status = VL53L1X_GetROI_XY(ToF_ADDRESS, &data, Y);
	sprintf(printf_buffer, "ROI set to: %u, %u", data, *Y);
	UART_printf(printf_buffer);*/
}

// Setting up Port J to receive interrupt events
void PortJ_Interrupt_Init(void){
	GPIO_PORTJ_IS_R = 0x0;		// PJ0-PJ1 is Edge-sensitive 
	GPIO_PORTJ_IBE_R = 0x0; 	// PJ0-PJ1 is not triggered by both edges 
	GPIO_PORTJ_IEV_R = 0x0;		// PJ0-PJ1 is falling edge event 
	GPIO_PORTJ_ICR_R = 0x3;		// Clear interrupt flag by setting proper bit in ICR register
	GPIO_PORTJ_IM_R = 0x3;		// Arm interrupt on PJ0-PJ1 by setting proper bit in IM register
	
	NVIC_EN1_R = 0x00080000;	// Enable interrupt 51 in NVIC (which is in Register EN1)
	
	NVIC_PRI12_R = 0x20000000;	// Set interrupt priority to 1
}

// Setting up the hardware timer
void Timer3_Init(void){
	// Step 1: Activate timer
	SYSCTL_RCGCTIMER_R = 0x08;		// (Step 1)Activate timer 
	SysTick_Wait10ms(1);			// Wait for the timer module to turn on
	
	// Step 2: Arm and Configure Timer Module
	TIMER3_CTL_R = 0x0;				// (Step 2.1) Disable Timer3 during setup (Timer stops counting)
	TIMER3_CFG_R = 0x00;			// (Step 2.2) Configure for 32-bit timer mode
	TIMER3_TAMR_R = 0x2;			// (Step 2.3) Configure for periodic mode
	TIMER3_TAPR_R = 0x0;			// (Step 2.4) Set prescale value to 0; i.e. Timer3 works with Maximum Freq = bus clock freq (36MHz)
	TIMER3_TAILR_R = 18000-1;		// (Step 2.5) Reload value (we multiply the period by 36 to match the units of 1 us)
	TIMER3_ICR_R = 0x1;				// (Step 2.6) Acknowledge the timeout interrupt (Clear timeout flag of Timer3)
	TIMER3_IMR_R = 0x1;				// (Step 2.7) Arm timeout interrupt


	// Step 3: Enable Interrupt at Processor side
	NVIC_EN1_R = 0x8;				// Enable IRQ 35 in NVIC
	NVIC_PRI8_R = 0x40000000;		// Set Interrupt Priority to 2

	// Step 4: Enable the Timer to start counting
	TIMER3_CTL_R = 0x1;				// Enable Timer3
} 


// This is the Interrupt Service Routine (ISR). This must be included and match the
// interrupt naming convention in startup_msp432e401y_uvision.s
// (Note - not the same as Valvano textbook).
void TIMER3A_IRQHandler(void){ 
	
	TIMER3_ICR_R =0x1;	// Acknowledge Timer3 timeout
	GPIO_PORTM_DATA_R ^= 0b00000001;
}

// Initiallize all ports, sensors, and interrupts
void initAll() {
	PLL_Init();				// Default Set System Clock to 120MHz
	SysTick_Init();			// Initialize SysTick configuration

	PortJ_Init();
	PortJ_Interrupt_Init();
	//PortM_Init();
	//PortM_Interrupt_Init();

	PortF_Init();
	PortM_Init();
	PortN_Init();
	PortH_Init();
	EnableInt();			// Enable Global Interrupts
	I2C_Init();
	UART_Init();
	ToF_Init();
	Timer3_Init();
}