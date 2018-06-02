/*
 * DAC output using software trigger to the DAC buffer
 * DAC has only two elements so update is done in ping-pong fashion
 */
#include "teensy_lc.h"
#include "fsl_dac_driver.h"

#define DAC_INSTANCE				(0)
#define DAC_DATA_SIZE				(1<<4)
#define DAC_DIDX_FLAG				(DAC_DATA_SIZE - 1)

// DAC data storage
uint16_t dac_data[DAC_DATA_SIZE];
// DAC data index
uint8_t dac_didx;

void dac_init(void);
extern void DAC0_IRQHandler(void);

int main(void)
{
	board_init();
	dbg_init(0, 115200);
	dac_init();

	PRINTF("\r\nDAC initialize.");

	while(1)
	{
		PRINTF("\r\n DAC data[%d]: %d",dac_didx,dac_data[dac_didx]);
		// give s/W trigger to the DAC buffer
		DAC_DRV_SoftTriggerBuffCmd(DAC_INSTANCE);

		GETCHAR();
	}

	return 0;
}


void dac_init()
{
	CLOCK_SYS_EnablePortClock(PORTE_IDX);
	// DAC out pin
	PORT_HAL_SetMuxMode(PORTE, 30u, kPortPinDisabled);

    dac_converter_config_t dacUserConfig;
    // take default configuration
    DAC_DRV_StructInitUserConfigNormal(&dacUserConfig);
    // initialize the DAC Converter.
    DAC_DRV_Init(DAC_INSTANCE, &dacUserConfig);

	// set up the DAC buffer
	dac_buffer_config_t dacBuffConfig;
	dacBuffConfig.bufferEnable = true;
	dacBuffConfig.triggerMode = kDacTriggerBySoftware;
	dacBuffConfig.idxStartIntEnable = true;
	dacBuffConfig.idxUpperIntEnable = true;
	dacBuffConfig.dmaEnable = false;
	dacBuffConfig.upperIdx = FSL_FEATURE_DAC_BUFFER_SIZE - 1;
	dacBuffConfig.buffWorkMode = kDacBuffWorkAsNormalMode;
	DAC_DRV_ConfigBuffer(DAC_INSTANCE, &dacBuffConfig);

	// fill data area
	uint16_t value = 0, i = 0;

	while(i < DAC_DATA_SIZE)
	{
		dac_data[i++] = value;
		value = value + 4095 / DAC_DATA_SIZE;
	}
	dac_didx = 0;
}


void DAC0_IRQHandler(void)
{
	// start flag raised
	if(DAC_DRV_GetBuffFlag(DAC_INSTANCE, kDacBuffIndexStartFlag))
	{
		DAC_DRV_ClearBuffFlag(DAC_INSTANCE, kDacBuffIndexStartFlag);
		// fill upper position of the DAC buffer
		DAC_HAL_SetBuffValue(DAC0,1,dac_data[dac_didx]);
		// advance data index
		dac_didx = (dac_didx+1) & DAC_DIDX_FLAG;
	}
	// upper flag raised
	else if(DAC_DRV_GetBuffFlag(DAC_INSTANCE, kDacBuffIndexUpperFlag))
	{
		DAC_DRV_ClearBuffFlag(DAC_INSTANCE, kDacBuffIndexUpperFlag);
		// fill start position of the DAC buffer
		DAC_HAL_SetBuffValue(DAC0,0,dac_data[dac_didx]);
		// advance data index
		dac_didx = (dac_didx+1) & DAC_DIDX_FLAG;
	}
	else
	{
		// unexpected IRQ
	}
}
