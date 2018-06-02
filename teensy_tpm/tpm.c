#include "teensy_lc.h"
#include "fsl_gpio_driver.h"
#include "fsl_tpm_driver.h"

enum _gpio_pins
{
	kGpioPBtn1 = GPIO_MAKE_PIN(PORTC_IDX, 7u),
	kGpioLED1 = GPIO_MAKE_PIN(PORTC_IDX, 5u),
};

void gpio_init(void);

#define TPM_MODULE			(TPM0)
#define TPM_INSTANCE		(0)

void tpm_init(void);
extern void TPM0_IRQHandler(void);

volatile bool tpmIsrFlag = false;


int main(void)
{
	board_init();
	gpio_init();
	tpm_init();

	while(1)
	{
		if(tpmIsrFlag)
		{
			GPIO_DRV_TogglePinOutput(kGpioLED1);
			tpmIsrFlag = false;
		}
	}

	return 0;
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


void tpm_init()
{
	// tpm module clock selection: SIM_SOPT2(PLLFLLSEL)
	CLOCK_SYS_SetTpmSrc(TPM_INSTANCE, kClockTpmSrcPllFllSel);
	// enable tpm module clock: redundant
	//CLOCK_SYS_EnableTpmClock(TPM_INSTANCE);

	// TPM general config
	tpm_general_config_t tpm_cfg = {
		.isDBGMode = false,
		.isGlobalTimeBase = false,
		.isTriggerMode = false,
		.isStopCountOnOveflow = false,
		.isCountReloadOnTrig = true,
		.triggerSource = kTpmTrigSel0
	};
	// tpm module initialization
	TPM_DRV_Init(TPM_INSTANCE, &tpm_cfg);
	// tpm clock selection: SIM_SOPT2(TPMSRC) - 375KHz, 2.67usec
	TPM_DRV_SetClock(TPM_INSTANCE, kTpmClockSourceModuleClk, kTpmDividedBy128);
	// tpm start counter - 750Hz, 1.33msec
	TPM_DRV_CounterStart(TPM_INSTANCE, kTpmCountingUp, 500U, true);
}


void TPM0_IRQHandler(void)
{
	TPM_HAL_ClearTimerOverflowFlag(TPM_MODULE);
	tpmIsrFlag = true;
}
