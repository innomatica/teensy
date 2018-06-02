#include "teensy_lc.h"
#include "fsl_gpio_driver.h"

enum _gpio_pins
{
	kGpioPBtn1 = GPIO_MAKE_PIN(PORTC_IDX, 7u),
	kGpioLED1 = GPIO_MAKE_PIN(PORTC_IDX, 5u),
};

void gpio_init(void);

int main(void)
{
	board_init();
	gpio_init();
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

void gpio_init(void)
{
	// enable port C clock
	CLOCK_SYS_EnablePortClock(PORTC_IDX);	// SIM_SCGC5(PORTA)

	// GPIO pushbutton pin config
	gpio_input_pin_user_config_t input_pin[] =
	{
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
	gpio_output_pin_user_config_t output_pin[] =
	{
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
