
#include <stdio.h>
#include <stdint.h>

#include "fsl_device_registers.h"
#include "fsl_clock_manager.h"
#include "fsl_gpio_driver.h"
#include "fsl_smc_hal.h"

enum _gpio_pins
{
	kGpioPBtn1 = GPIO_MAKE_PIN(PORTC_IDX, 7u),
	kGpioLED1 = GPIO_MAKE_PIN(PORTC_IDX, 5u),
};

void board_init(void);
void clock_init(void);
void gpio_init(void);
void enable_clkout(int clktype);

int main(void)
{
	board_init();
	enable_clkout(kClockClkoutBusClk);

	while(1)
	{
		if(GPIO_DRV_ReadPinInput(kGpioPBtn1))
			GPIO_DRV_ClearPinOutput(kGpioLED1);
		else
			GPIO_DRV_SetPinOutput(kGpioLED1);
	}

	return 0;
}


void board_init(void)
{
	clock_init();
	gpio_init();
}

void clock_init(void)
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

void gpio_init(void)
{
	// enable port A clock
	CLOCK_SYS_EnablePortClock(PORTC_IDX);	// SIM_SCGC5(PORTA)

	// GPIO pushbutton pin config
	gpio_input_pin_user_config_t input_pin[] = {
		{
			.pinName = kGpioPBtn1,
			.config.isPullEnable = true,
			.config.pullSelect = kPortPullUp,
			.config.interrupt = kPortIntDisabled,
		},
		{
			.pinName = GPIO_PINS_OUT_OF_RANGE,
		}
	};

	// GPIO LED pin config
	gpio_output_pin_user_config_t output_pin[] = {
		{
			.pinName = kGpioLED1,
			.config.outputLogic = 0,
			.config.slewRate = kPortFastSlewRate,
			.config.driveStrength = kPortHighDriveStrength,
		},
		{
			.pinName = GPIO_PINS_OUT_OF_RANGE,
		}
	};

	GPIO_DRV_Init(input_pin, output_pin);
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
