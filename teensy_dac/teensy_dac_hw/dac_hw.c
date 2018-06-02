/*
 * TPM -> DMA -> DAC
 *
 *  TPM triggers DMA
 *  DMA moves 16bit data from one buffer memory to DAC
 *  At the end of transfer of DMA:
 *		DMA switches source buffer from one buffer to the other
 */
#include "teensy_lc.h"
#include "fsl_gpio_driver.h"
#include "fsl_dac_driver.h"
#include "fsl_dma_driver.h"
#include "fsl_tpm_driver.h"
#include "fsl_os_abstraction.h"

// audio ping pong buffer
#define AUDIO_BSIZE					(1<<4)
uint16_t audioBuff1[AUDIO_BSIZE];
uint16_t audioBuff2[AUDIO_BSIZE];
uint8_t idxBuff = 0;
void fill_buffer();

// GPIO
enum _gpio_pins
{
	kGpioDI1 = GPIO_MAKE_PIN(PORTC_IDX, 2u),
	kGpioDI2 = GPIO_MAKE_PIN(PORTD_IDX, 5u),
	kGpioDO1 = GPIO_MAKE_PIN(PORTD_IDX, 7u),
	kGpioDO2 = GPIO_MAKE_PIN(PORTD_IDX, 4u),
};
void gpio_init(void);

// DAC
#define DAC_MODULE					(DAC0)
#define DAC_INSTANCE				(0)
void dac_init(void);
extern void DAC0_IRQHandler(void);

// DMA
#define DMA_MODULE					(DMA0)
#define DMA_INSTANCE				(0)
#define DMA_CHNSPK					(0)
dma_state_t dmaState = {{0}};	// double bracket suppresses gcc warning
dma_channel_t dmaChnSpk;
void dma_init();
void dma_deinit();
void DMA_CB_Spk(void *param, dma_channel_status_t chanStatus);
extern void DMA0_IRQHandler(void);

// TPM
#define TPM_MODULE					(TPM0)
#define TPM_INSTANCE				(0)
void tpm_init(void);
extern void TPM0_IRQHandler(void);

// OSA semaphore
semaphore_t sema;


int main(void)
{
	board_init();
	dbg_init(0, 115200);

	// activate OSA layer
	OSA_Init();
	// create a semaphore
	OSA_SemaCreate(&sema, 0);

	// initialize gpio
	gpio_init();
	// initialize DAC
	dac_init();
	// fill test data
	fill_buffer();
	// initialize dma
	dma_init();

	// show registers
	PRINTF("\r\nDMA_DCR0: %02X.%02X.%02X.%02X",
			(uint8_t)(DMA_RD_DCR(DMA_MODULE,DMA_CHNSPK)>>24),
			(uint8_t)(DMA_RD_DCR(DMA_MODULE,DMA_CHNSPK)>>16),
			(uint8_t)(DMA_RD_DCR(DMA_MODULE,DMA_CHNSPK)>>8),
			(uint8_t)(DMA_RD_DCR(DMA_MODULE,DMA_CHNSPK)));
	PRINTF("\r\nDMA_DSR_BCR0: %02X.%02X.%02X.%02X",
			(uint8_t)(DMA_RD_DSR_BCR(DMA_MODULE,DMA_CHNSPK)>>24),
			(uint8_t)(DMA_RD_DSR_BCR(DMA_MODULE,DMA_CHNSPK)>>16),
			(uint8_t)(DMA_RD_DSR_BCR(DMA_MODULE,DMA_CHNSPK)>>8),
			(uint8_t)(DMA_RD_DSR_BCR(DMA_MODULE,DMA_CHNSPK)));
	PRINTF("\r\nDMAMUX0_CHCFG0: %02X", (uint8_t)(DMAMUX0_CHCFG0));

	// initialize TPM: this will initiate the DMA automatically
	tpm_init();

	while(1)
	{
		if(GPIO_DRV_ReadPinInput(kGpioDI1) == 0)
		{
			// pusubutton 1 is depressed
			// interrupt is not necessary
			TPM_DRV_CounterStart(TPM_INSTANCE, kTpmCountingUp, 4096U, true);
		}
		else if(GPIO_DRV_ReadPinInput(kGpioDI2) == 0)
		{
			// pushbutton 2 is depressed
			TPM_DRV_CounterStop(TPM_INSTANCE);
		}

		// check semaphore
		if(kStatus_OSA_Success == OSA_SemaWait(&sema, 0))
			fill_buffer();
	}

	PRINTF("\r\nDMA transfer done------------------------------");

	PRINTF("\r\nDMA_DSR_BCR0: %02X.%02X.%02X.%02X\r\n",
			(uint8_t)(DMA_RD_DSR_BCR(DMA_MODULE,DMA_CHNSPK)>>24),
			(uint8_t)(DMA_RD_DSR_BCR(DMA_MODULE,DMA_CHNSPK)>>16),
			(uint8_t)(DMA_RD_DSR_BCR(DMA_MODULE,DMA_CHNSPK)>>8),
			(uint8_t)(DMA_RD_DSR_BCR(DMA_MODULE,DMA_CHNSPK)));

	// clean up the channel
	dma_deinit();
	OSA_SemaDestroy(&sema);

	return 0;
}


void dac_init()
{
	// configure DAC out pin
	CLOCK_SYS_EnablePortClock(PORTE_IDX);
	PORT_HAL_SetMuxMode(PORTE, 30u, kPortPinDisabled);

	// DAC setting
    dac_converter_config_t dacUserConfig;
    // set default configuration
    DAC_DRV_StructInitUserConfigNormal(&dacUserConfig);
    // initialize the DAC Converter with the default setting.
    DAC_DRV_Init(DAC_INSTANCE, &dacUserConfig);
}


void dma_init()
{
	// initialize DMA module
    DMA_DRV_Init(&dmaState);
	// request a channel 0 with TPM0 OVF trigger
	DMA_DRV_RequestChannel(DMA_CHNSPK,kDmaRequestMux0TPM0Overflow,&dmaChnSpk);
	//DMA_DRV_RequestChannel(DMA_CHNSPK,kDmaRequestMux0TPM0Channel0,&dmaChnSpk);

	// Configure a channel.
	//
	// This function sets DMA_DREQ regester with the following values
	// in addition to set the source address (DMA_SARn), the destination
	// address (DMA_DARn), and byte count (DMA_DSR_BCRn). Thus further
	// adjustment of parameters may be necessary.
	//
	// X denotes that actual values vary depending on other parameters
	//
	// EINT(1): enable interrupt on completion (DMA_HAL_SetIntCmd)
	// ERQ(0): enable peripheral request (DMA_HAL_SetDmaRequestCmd)
	// CS(1): cycle steal mode (DMA_HAL_SetCycleStealCmd)
	// AA(0): auto-align (DMA_HAL_SetAutoAlignCmd)
	// DADREQ(0): enable asynchronous DMA (DMA_HAL_SetAsyncDmaRequestCmd)
	// SINC(X): source increment (DMA_HAL_SetSourceIncrementCmd)
	// SSIZE(X): source size (DMA_HAL_SetSourceTransferSize)
	// DINC(X): destination increment (DMA_HAL_SetDestIncrementCmd)
	// DSIZE(X): destination size (DMA_HAL_SetDestTransferSize)
	// START(0): start transfer (DMA_HAL_SetTriggerStartCmd)
	// SMOD(0): source address modulo (DMA_HAL_SetSourceModulo)
	// DMOD(0): destination address modulo (DMA_HAL_SetDestModulo)
	// D_REQ(1): disable requet (DMA_HAL_SetDisableRequestAfterDoneCmd)
	// LINKCC(0): link channel control (DMA_HAL_SetChanLink)
	// LCH1(0): link channel 1
	// LCH2(0): link channel 2
	//
	DMA_DRV_ConfigTransfer(&dmaChnSpk, kDmaMemoryToPeripheral, 2,
			(uint32_t)audioBuff1, (uint32_t)(&DAC0_DATL(0)), AUDIO_BSIZE*2);

	// keep running after done
	DMA_HAL_SetDisableRequestAfterDoneCmd(DMA_MODULE,DMA_CHNSPK,false);
	// register dma callback
	DMA_DRV_RegisterCallback(&dmaChnSpk, DMA_CB_Spk, NULL);
	// start channel
	DMA_DRV_StartChannel(&dmaChnSpk);
}


void dma_deinit()
{
	DMA_DRV_StopChannel(&dmaChnSpk);
	DMA_DRV_FreeChannel(&dmaChnSpk);
	DMA_DRV_Deinit();
}


void gpio_init()
{
	// enable port D clock
	CLOCK_SYS_EnablePortClock(PORTD_IDX);

	// GPIO DI pin config
	gpio_input_pin_user_config_t input_pin[] =
	{
		{
			.pinName = kGpioDI1,
			.config.isPullEnable = true,
			.config.pullSelect = kPortPullUp,
			.config.interrupt = kPortIntDisabled,
		},
		{
			.pinName = kGpioDI2,
			.config.isPullEnable = true,
			.config.pullSelect = kPortPullUp,
			.config.interrupt = kPortIntDisabled,
		},
		{
			.pinName = GPIO_PINS_OUT_OF_RANGE,
		}
	};
	// GPIO DO pin config
	gpio_output_pin_user_config_t output_pin[] =
	{
		{
			.pinName = kGpioDO1,
			.config.outputLogic = 0,
			.config.slewRate = kPortFastSlewRate,
			.config.driveStrength = kPortHighDriveStrength,
		},
		{
			.pinName = kGpioDO2,
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
	// enable tpm module clock: this is redundant
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
	// tpm clock selection: SIM_SOPT2(TPMSRC): 48MHz
	TPM_DRV_SetClock(TPM_INSTANCE, kTpmClockSourceModuleClk, kTpmDividedBy1);
	//
	// enable DMA trigger
	//
	// Note that this feature is not exposed via HAL nor Peripheral Library
	// as of this writing. Thus direct call is necessary here.
	//
	TPM_WR_SC_DMA(TPM_MODULE, 1);
	// tpm start counter: 11.718KHz
	//TPM_DRV_CounterStart(TPM_INSTANCE, kTpmCountingUp, 4096U, true);
}


void fill_buffer()
{
	uint16_t delta = (uint16_t)(4096. / AUDIO_BSIZE);

	if((idxBuff == 1)||(idxBuff == 0))
	{
		// fill second buffer
		audioBuff2[0] = 4000;
		for(int i = 1; i < AUDIO_BSIZE; i++)
		{
			audioBuff2[i] = audioBuff2[i-1] - delta;
		}
	}
	else if((idxBuff == 2)||(idxBuff == 0))
	{
		// fill first buffer
		audioBuff1[0] = 0;
		for(int i = 1; i < AUDIO_BSIZE; i++)
		{
			audioBuff1[i] = audioBuff1[i-1] + delta;
		}
	}

	if(idxBuff == 0)
		idxBuff = 1;
}


/*
 * End of DMA transfer callback
 */
void DMA_CB_Spk(void *param, dma_channel_status_t chanStatus)
{
	// clear DMA status
	DMA_HAL_ClearStatus(DMA_MODULE, DMA_CHNSPK);
	// toggle buffer
	if(idxBuff == 1)
	{
		DMA_HAL_SetSourceAddr(DMA_MODULE, DMA_CHNSPK, (uint32_t)audioBuff2);
		idxBuff = 2;
	}
	else
	{
		DMA_HAL_SetSourceAddr(DMA_MODULE, DMA_CHNSPK, (uint32_t)audioBuff1);
		idxBuff = 1;
	}
	// reset count
	DMA_BWR_DSR_BCR_BCR(DMA_MODULE, DMA_CHNSPK, AUDIO_BSIZE * 2);

	// raise semaphone
	OSA_SemaPost(&sema);
}


/*
 * DMA IRQ handler
 */
void DMA0_IRQHandler(void)
{
    DMA_DRV_IRQhandler(0);
	GPIO_DRV_TogglePinOutput(kGpioDO2);
}

/*
 * DAC IRQ handler
 */
/*
void DAC0_IRQHandler(void)
{
	DAC_DRV_IRQhandler(0);
}
*/

/*
 * TPM IRQ Handler
 *
 * This interrupt service routine is here only for the verification
 * To activate this interrupt use 'true' when calling
 *
 *	TPM_DRV_CounterStart(TPM_INSTANCE, kTpmCountingUp, 4096U, false);
 *
 */
void TPM0_IRQHandler(void)
{
	TPM_HAL_ClearTimerOverflowFlag(TPM_MODULE);
	GPIO_DRV_TogglePinOutput(kGpioDO1);
}
