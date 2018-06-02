/*
 * ADC single conversion polling example
 *	no interrupt
 *	no hardware trigger
 *	no continuous conversion
 *	time base is set by PIT
 */
#include "teensy_lc.h"
#include "fsl_gpio_driver.h"
#include "fsl_pit_driver.h"
#include "fsl_adc16_driver.h"

enum _gpio_pins
{
	kGpioPBtn1 = GPIO_MAKE_PIN(PORTC_IDX, 7u),
	kGpioLED1 = GPIO_MAKE_PIN(PORTC_IDX, 5u),
};


#define ADC_INST			(0)		// ADC instacne
#define ADC_CHAN			(14)	// ADC channel
#define ADC_CGRP			(0)		// ADC channel group

volatile bool pitIsrFlag = false;

void gpio_init(void);
void pit_init(void);
void adc_init(uint32_t adc_no);
uint32_t adc_measure(void);


int main(void)
{
	board_init();
	gpio_init();
	adc_init(ADC_INST);
	pit_init();
	dbg_init(0, 115200);

	while(1)
	{
		if(pitIsrFlag)
		{
			GPIO_DRV_TogglePinOutput(kGpioLED1);
			// display adc value
			PRINTF("\r\n ADC converted value: %d", adc_measure() );
			// reset PIT int flag
			pitIsrFlag = false;
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


void adc_init(uint32_t adc_no)
{
	// initialize the module with user parameter
	adc16_converter_config_t adcUserConfig = {	// ADC configuration
		.lowPowerEnable = false,							// ADCx_CFG1(ADLPC)
		.clkDividerMode = kAdc16ClkDividerOf1,				// ADCx_CFG1(ADIV)
		.longSampleTimeEnable = true,						// ADCx_CFG1(ADLSMP)
		.resolution = kAdc16ResolutionBitOf16,				// ADCx_CFG1(MODE)
		.clkSrc = kAdc16ClkSrcOfAsynClk,					// ADCx_CFG1(ADICLK)
		.asyncClkEnable = true,								// ADCx_CFG2(ADACKEN)
		.highSpeedEnable = false,							// ADCx_CFG2(ADHSC)
		.longSampleCycleMode = kAdc16LongSampleCycleOf6,	// ADCx_CFG2(ADLSTS)
		.hwTriggerEnable = false,							// ADCx_SC2(ADTRG)
		.refVoltSrc = kAdc16RefVoltSrcOfVref,				// ADCx_SC2(REFSEL)
		.continuousConvEnable = false,						// ADCx_SC3(ADC0)
		.dmaEnable = false									// ADCx_SC2(DMAEN)
	};
    ADC16_DRV_Init(ADC_INST, &adcUserConfig);

    // perform auto calibration
    adc16_calibration_param_t adcCalibraitionParam;
    ADC16_DRV_GetAutoCalibrationParam(ADC_INST, &adcCalibraitionParam);
    ADC16_DRV_SetCalibrationParam(ADC_INST, &adcCalibraitionParam);

    // Use hardware average to increase stability of the measurement.
    adc16_hw_average_config_t userHwAverageConfig = {
		.hwAverageEnable = true,						// ADCx_SC3(AVGE)
		.hwAverageCountMode = kAdc16HwAverageCountOf4	// ADCx_SC3(AVGS)
	};
    ADC16_DRV_ConfigHwAverage(ADC_INST, &userHwAverageConfig);
}


uint32_t adc_measure(void)
{
	uint32_t adcValue;

    adc16_chn_config_t chnConfig = {	// channel configuration
		.chnIdx = (adc16_chn_t)ADC_CHAN,		// ADCx_SC1n(ADCH)
		.diffConvEnable = false,				// ADCx_SC1n(DIFF)
		.convCompletedIntEnable = false			// ADCx_SC1n(AIEN)
	};

    // Configuration triggers the conversion in software conversion case
    ADC16_DRV_ConfigConvChn(ADC_INST, ADC_CGRP, &chnConfig);
    // Wait for the conversion to be done.
    ADC16_DRV_WaitConvDone(ADC_INST, ADC_CGRP);
    // Fetch the conversion value.
    adcValue = ADC16_DRV_GetConvValueRAW(ADC_INST, ADC_CGRP);
    // Pause the conversion.
    ADC16_DRV_PauseConv(ADC_INST, ADC_CGRP);

	return adcValue;
}


void pit_init(void)
{
	// PIT channel config
	const pit_user_config_t pitCh0Config = {
		.isInterruptEnabled = true,
		.periodUs = 1000000U
	};
	// initiallize PIT module
	PIT_DRV_Init(0, false);
	// initialize pit channel 0
	PIT_DRV_InitChannel(0,0, &pitCh0Config);
	// start PIT0
	PIT_DRV_StartTimer(0,0);
}


/*
 * PIT IRQ handler
 */
void PIT_IRQHandler(void)
{
	// clear interrupt flag
	PIT_HAL_ClearIntFlag(PIT,0);
	// set the flag for the main to use
	pitIsrFlag = true;
}
