#include "teensy_lc.h"


int main(void)
{
	board_init();

	if (dbg_init(0, 115200))
	{
		// print test
		PRINTF("\r\nDebug Console activated.\r\n");
		PRINTF("System Clock: %d\r\n", CLOCK_SYS_GetSystemClockFreq());
		PRINTF("Bus Clock: %d\r\n", CLOCK_SYS_GetBusClockFreq());
		PRINTF("Flash Clock: %d\r\n", CLOCK_SYS_GetFlashClockFreq());
		PRINTF("Fixed Freq Clock: %d\r\n", CLOCK_SYS_GetFixedFreqClockFreq());
		PRINTF("Fll Clock: %d\r\n", CLOCK_HAL_GetFllClk(MCG));
		PRINTF("Pll0 Clock: %d\r\n", CLOCK_HAL_GetPll0Clk(MCG));
		PRINTF("PllFll Clock: %d\r\n", CLOCK_SYS_GetPllFllClockFreq());
		PRINTF("IR Clock: %d\r\n", CLOCK_HAL_GetInternalRefClk(MCG));
		PRINTF("Lpo Clock: %d\r\n", CLOCK_SYS_GetLpoClockFreq());
		PRINTF("Systick Clock: %d\r\n", CLOCK_SYS_GetSystickFreq());
	}

	while(1)
	{
	}

	return 0;
}
