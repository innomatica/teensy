#include "teensy_lc.h"
#include "fsl_gpio_driver.h"
#include "fsl_pit_driver.h"

enum _gpio_pins
{
	kGpioPBtn1 = GPIO_MAKE_PIN(PORTC_IDX, 7u),
	kGpioLED1 = GPIO_MAKE_PIN(PORTC_IDX, 5u),
};

volatile bool pitIsrFlag = false;

void gpio_init(void);
void pit_init(void);

int main(void)
{
	board_init();
	gpio_init();
	pit_init();

	enable_clkout(kClockClkoutMcgIrClk);

	while(1)
	{
		if(pitIsrFlag)
		{
			GPIO_DRV_TogglePinOutput(kGpioLED1);
			pitIsrFlag = false;
		}
	}

	return 0;
}


void gpio_init(void)
{
	// enable port C clock
	CLOCK_SYS_EnablePortClock(PORTC_IDX);	// SIM_SCGC5(PORTC)

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


void pit_init(void)
{
	// PIT channel config
	const pit_user_config_t pitCh0Config = {
		.isInterruptEnabled = true,
		.periodUs = 2000U
	};
	// initiallize PIT module
	PIT_DRV_Init(0, false);
	// initialize pit channel 0
	PIT_DRV_InitChannel(0,0, &pitCh0Config);
	// start PIT0
	PIT_DRV_StartTimer(0,0);
}


void PIT_IRQHandler(void)
{
	PIT_HAL_ClearIntFlag(PIT,0);
	pitIsrFlag = true;
}
