/*
 * ADC hardware trigger example
 *	PIT trigger
 *	interrupt after CC
 */
#include "teensy_lc.h"
#include "fsl_gpio_driver.h"
#include "fsl_pit_driver.h"
#include "fsl_adc16_driver.h"
#include "fsl_pmc_hal.h"

enum _gpio_pins
{
	kGpioPBtn1 = GPIO_MAKE_PIN(PORTC_IDX, 7u),
	kGpioLED1 = GPIO_MAKE_PIN(PORTC_IDX, 5u),
};

#define ADC_INST			(0)		// ADC instacne
#define ADC_CHAN			(14)	// ADC channel
#define ADC_CGRP			(0)		// ADC channel group

volatile bool adcIsrFlag = false;
volatile uint16_t adcValue = 0;

void gpio_init(void);
void pit_init(void);
void adc_init(uint32_t adc_no);


int main(void)
{
	board_init();
	gpio_init();
	adc_init(ADC_INST);
	pit_init();
	dbg_init(0, 115200);

	while(1)
	{
		if(adcIsrFlag)
		{
			GPIO_DRV_TogglePinOutput(kGpioLED1);
			PRINTF("\r\nADC reading: %d", adcValue);
			adcIsrFlag = false;
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
		/*
		 * NOTE THAT 16BIT RESOLUTION SETTING WITH HARDWARE TRIGGER YIELDS
		 * FAULTY BEHAVIOUR OF THE ADC MODULE.
		 * IT IS NOT CERTAIN THIS IS A SILICON ERROR OR NOT.
		 */
		.resolution = kAdc16ResolutionBitOf12or13,			// ADCx_CFG1(MODE)
		.clkSrc = kAdc16ClkSrcOfAsynClk,					// ADCx_CFG1(ADICLK)
		.asyncClkEnable = true,								// ADCx_CFG2(ADACKEN)
		.highSpeedEnable = false,							// ADCx_CFG2(ADHSC)
		.longSampleCycleMode = kAdc16LongSampleCycleOf6,	// ADCx_CFG2(ADLSTS)
		.hwTriggerEnable = true,							// ADCx_SC2(ADTRG)
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

	// channel configuration
    adc16_chn_config_t chnConfig = {	// channel configuration
		.chnIdx = (adc16_chn_t)ADC_CHAN,			// ADCx_SC1n(ADCH)
		.diffConvEnable = false,					// ADCx_SC1n(DIFF)
		.convCompletedIntEnable = true				// ADCx_SC1n(AIEN)
	};

    // Configuration triggers the conversion in software conversion case
	// but not here.
    ADC16_DRV_ConfigConvChn(ADC_INST, ADC_CGRP, &chnConfig);
}


void pit_init(void)
{
	// PIT channel config
	const pit_user_config_t pitCh0Config = {
		.isInterruptEnabled = false,
		.periodUs = 500000U
	};
	// initiallize PIT module
	PIT_DRV_Init(0, true);
	// initialize pit channel 0
	PIT_DRV_InitChannel(0,0, &pitCh0Config);

	// start PIT0
	PIT_DRV_StartTimer(0,0);

	// configure SIM for ADC hw trigger source: SIM_SOPT7
	SIM_HAL_SetAdcAlternativeTriggerCmd(SIM, ADC_INST, true);
	// Trigger Selection A corresponds to channel group 0(SC1A)
	SIM_HAL_SetAdcPreTriggerMode(SIM, ADC_INST, kSimAdcPretrgselA);
	SIM_HAL_SetAdcTriggerMode(SIM, ADC_INST, kSimAdcTrgSelPit0);
}



/*
 * ADC IRQ Handler
 */
void ADC0_IRQHandler(void)
{
	// COCO flag check is unnecessary for this example
	//if(ADC16_DRV_GetChnFlag(ADC_INST, ADC_CGRP, kAdcChnConvCompleteFlag))
	{
		adcValue = ADC16_DRV_GetConvValueRAW(ADC_INST, ADC_CGRP);
		adcIsrFlag = true;
	}

}
