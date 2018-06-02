#include "teensy_lc.h"

/*
 * teensy lc board initialization
 *
 *	mcg mode	: PEE
 *	crystal		: 16MHz
 *	core clock	: 48MHz
 *	bus clock	: 24MHz
 *	ir clock	: 32KHz
 *	osc clock	: 16MHz
 *	lpo clock	: 1KHz
 */
void board_init(void)
{
	// enable port A clock
	CLOCK_SYS_EnablePortClock(PORTA_IDX);	// SIM_SCGC5(PORTA)

	// set power modes
	SMC_HAL_SetProtection(SMC, kAllowPowerModeAll);	// SMC_PMPROT

	// setup external OSC pins
	PORT_HAL_SetMuxMode(PORTA, 18, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTA, 19, kPortPinDisabled);

	// initialize OSC
	osc_user_config_t osc_cfg =
	{
		.freq                = 16000000U,			// OSC 16MHz
		.hgo                 = kOscGainLow,			// MCG_C2(HGO0)
		.range               = kOscRangeVeryHigh,	// MCG_C2(RANGE0)
		.erefs               = kOscSrcOsc,			// MCG_C2(EREFS0)
		.enableCapacitor2p   = true,				// OSC0_CR(SC2P)
		.enableCapacitor4p   = true,				// OSC0_CR(SC4P)
		.enableCapacitor8p   = false,				// OSC0_CR(SC8P)
		.enableCapacitor16p  = false,				// OSC0_CR(SC16P)
	};
	CLOCK_SYS_OscInit(0U, &osc_cfg);

	sim_config_t sim_cfg =
	{
		.pllFllSel = kClockPllFllSelPll,    // SIM_SOPT2(PLLFLLSEL)
		.er32kSrc  = kClockEr32kSrcLpo,     // SIM_SOPT1(OSC32KSEL)
		.outdiv1   = 1U,					// SIM_CLKDIV1(OUTDIV1):48MHz
		.outdiv4   = 1U,					// SIM_CLKDIV1(OUTDIV2):24MHz
	};
	CLOCK_SYS_SetSimConfigration(&sim_cfg);

	oscer_config_t oer_cfg =
	{
		.enable       = true,				// OSC0_CR(ERCLKEN)
		.enableInStop = false,				// OSC0_CR(EREFSTEN)
	};
	CLOCK_SYS_SetOscerConfigration(0U, &oer_cfg);

	mcg_config_t mcg_cfg =
	{
		.mcg_mode = kMcgModePEE,			// MCG mode

		.irclkEnable = true,				// MCG_C1(IRCLKEN)
		.irclkEnableInStop = false,			// MCG_C1(IREFSTEN)
		.ircs = kMcgIrcSlow,				// MCG_C2(IRCS)
		.fcrdiv = 0U,						// MCG_SC(FCRDIV)

		.frdiv = 4U,						// MCG_C1(FRDIV)

		.drs = kMcgDcoRangeSelMid,			// MCG_C4(DRST_DRS)
		.dmx32 = kMcgDmx32Fine,				// MCG_C4(DMX32)

		.pll0EnableInFllMode = false,		// MCG_C5(PLLCLKEN0)
		.pll0EnableInStop = false,			// MCG_C5(PLLSTEN0)
		.prdiv0 = 0x3U,						// MCG_C5(PRDIV0):PLL in 4MHz
		.vdiv0 = 0x0U,						// MCG_C6(VDIV0):PLL out 96MHz
	};
	CLOCK_SYS_BootToPee(&mcg_cfg);
	SystemCoreClock = 48000000u;
}


/*
 * PORTC3 will output selected internal clock
 *
 * possible clktype values are
 *	kClockClkoutBusClk
 *	kClockClkoutLpoClk
 *	kClockClkoutMcgIrClk
 *	kClockClkoutOsc0erClk
 */
void enable_clkout(int clktype)
{
	// enable port C clock
	CLOCK_SYS_EnablePortClock(PORTC_IDX);	// SIM_SCGC5(PORTC)

	// select CLKOUT for C3
	PORT_HAL_SetMuxMode(PORTC, 3, kPortMuxAlt5);

	// select MCGIRCLK as CLKOUT
	CLOCK_HAL_SetClkOutSel(SIM, clktype);
}


/*
 * Open a serial port for debug output. Instance 0 will use LPSCI driver.
 *
 * Pin allocations:
 *  Instance 0: PortA1, PortA2
 *  Instance 1: PortC3, PortC4
 *  Instance 2: PortD2, PortD3
 *
 * Arguments:
 *	inst: instance number of the UARTs 0,1, or 2
 *	baud: valid baud
 */
bool dbg_init(uint32_t inst, uint32_t baud)
{
	if(inst == 0)
	{
		// enable port A clock
		CLOCK_SYS_EnablePortClock(PORTA_IDX);	// SIM_SCGC5(PORTA)
		// UART0 port initialization
		PORT_HAL_SetMuxMode(PORTA,1,kPortMuxAlt2);	// UART0_RX
		PORT_HAL_SetMuxMode(PORTA,2,kPortMuxAlt2);	// UART0_TX

		// initialize module clock
		CLOCK_SYS_EnableLpsciClock(0);
		CLOCK_SYS_SetLpsciSrc(inst, kClockLpsciSrcPllFllSel);
		// initialize console
		DbgConsole_Init(inst, baud, kDebugConsoleLPSCI);

		return true;
	}
	else
	{
		if(inst == 1)
		{
			// enable port C clock
			CLOCK_SYS_EnablePortClock(PORTC_IDX);	// SIM_SCGC5(PORTC)
			// UART0 port initialization
			PORT_HAL_SetMuxMode(PORTC,3,kPortMuxAlt3);	// UART1_RX
			PORT_HAL_SetMuxMode(PORTC,4,kPortMuxAlt3);	// UART1_TX
		}
		else if(inst == 2)
		{
			// enable port D clock
			CLOCK_SYS_EnablePortClock(PORTD_IDX);	// SIM_SCGC5(PORTD)
			// UART0 port initialization
			PORT_HAL_SetMuxMode(PORTD,2,kPortMuxAlt3);	// UART2_RX
			PORT_HAL_SetMuxMode(PORTD,3,kPortMuxAlt3);	// UART2_TX
		}
		else
		{
			return false;
		}
		// enable module clock
		CLOCK_SYS_EnableUartClock(inst);
		// initialize console
		DbgConsole_Init(inst, baud, kDebugConsoleUART);

		return true;
	}
}
