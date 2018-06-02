/*
 *  TPM -> ADC -> DMA -> SPI
 *
 *	 TPM triggers ADC
 *	 ADC end of conversion triggers DMA
 *	 DMA moves 16bit data from ADC to one buffer memory
 *	 At the end of transfer of DMA:
 *		CPU moves data from one buffer to SPI EEPROM
 *		DMA switches destination buffer memory from one to the other
 */
#include "teensy_lc.h"
#include "at25m.h"
#include "tpa2016.h"

#include "fsl_adc16_driver.h"
#include "fsl_dac_driver.h"
#include "fsl_dma_driver.h"
#include "fsl_gpio_driver.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_os_abstraction.h"
#include "fsl_spi_master_driver.h"
#include "fsl_spi_shared_function.h"
#include "fsl_tpm_driver.h"

#define DEBUG_PRINT			(1)

// state machine and tasks
typedef enum {STATE_SLEEP, STATE_IDLE, STATE_PLAY, STATE_RECORD} State_Type;
State_Type prev_state, next_state;
void set_state_flag(void);
void task_sleep(void);
void task_idle(void);
void task_play(void);
void task_record(void);
void (*state_task[])(void) = {task_sleep, task_idle, task_play, task_record};

// audio ping pong buffer: integer division of EEPROM page size
#define AUDIO_BUFSIZE				(EEPROM_PAGESIZE>>2)
// actual buffer was augmented to hold SPI EEPROM command and address
uint8_t audioBuff1[AUDIO_BUFSIZE+4];
uint8_t audioBuff2[AUDIO_BUFSIZE+4];
#define BUFF_SHIFT					(4)
uint16_t audioBuff1X[AUDIO_BUFSIZE+4];
uint16_t audioBuff2X[AUDIO_BUFSIZE+4];
uint16_t idxAudioBuff = 1;						// buffer index


// ADC
#define ADC_MODULE					(ADC0)
#define ADC_INSTANCE				(0)		// ADC instacne
#define ADC_CHNMIC					(8)		// ADC channel
#define ADC_CHNGROUP				(0)		// ADC channel group
volatile bool adcIsrFlag = false;
volatile uint16_t adcValue = 0;
void adc_init();

// DAC
#define DAC_MODULE					(DAC0)
#define DAC_INSTANCE				(0)
void dac_init(void);
extern void DAC0_IRQHandler(void);

// DMA
#define DMA_MODULE					(DMA0)
#define DMA_INSTANCE				(0)
#define DMA_CHNMIC					(0)
#define DMA_CHNSPK					(1)
dma_state_t dmaState;
dma_channel_t dmaChnMic;
dma_channel_t dmaChnSpk;

void dma_init();
void dma_deinit();
void DMA_CB_Mic(void *param, dma_channel_status_t chanStatus);
void DMA_CB_Spk(void *param, dma_channel_status_t chanStatus);
extern void DMA0_IRQHandler(void);

// GPIO
enum _gpio_pins
{
	kGpioDI1 = GPIO_MAKE_PIN(PORTC_IDX, 2u),
	kGpioDI2 = GPIO_MAKE_PIN(PORTD_IDX, 5u),
	kGpioDO1 = GPIO_MAKE_PIN(PORTD_IDX, 7u),
	kGpioDO2 = GPIO_MAKE_PIN(PORTD_IDX, 4u),
	kGpioSS = GPIO_MAKE_PIN(PORTC_IDX, 4u),
};
void gpio_init(void);

// I2C Audio Amplifier
#define I2C_INSTANCE				(0)
#define DATA_LENGTH					(64)
i2c_master_state_t master;
i2c_device_t tpa2016 =
{
	.address = DEV_ADDR,
	.baudRate_kbps = 400   // 400 Kbps
};

uint8_t txBuff[DATA_LENGTH] = {0};
uint8_t rxBuff[DATA_LENGTH] = {0};

void i2c_init(void);
extern void I2C_DRV_IRQHandler(uint32_t instance);

// SPI EEPROM
#define SPI_INSTANCE				(0)
#define SPI_BAUD					(4000000u)		// 4MHz
#define SPI_TIMEOUT					(5000u)
#define EEPROM_HEADER				(0)
#define EEPROM_DATA					EEPROM_PAGESIZE
#define EEPROM_MAXADDR				(EEPROM_CAPACITY - EEPROM_PAGESIZE)

spi_master_state_t spiMasterState;
uint32_t addrEEPROM;
void spi_init(void);

// TPM
#define TPM_MODMIC					(TPM0)
#define TPM_MODSPK					(TPM1)
#define TPM_INSTMIC					(0)
#define TPM_INSTSPK					(1)
// FIXME: 4096 for 11KHz, 40960 for 1.1KHz
//#define TPM_FREQ					(4096u)
//#define TPM_DIV						(kTpmDividedBy1)
#define TPM_FREQ					(3000)
#define TPM_DIV						(kTpmDividedBy2)
void tpm_init(void);
extern void TPM0_IRQHandler(void);
extern void TPM1_IRQHandler(void);

// OSA semaphore
semaphore_t sema;


int main(void)
{
	// initialize CPU
	board_init();
#if DEBUG_PRINT
	// initialize debug monitor
	dbg_init(0, 115200);
#endif

	// activate OSA layer
	OSA_Init();

	// initialize modules
	gpio_init();
	adc_init();
	dac_init();
	dma_init();
	i2c_init();
	spi_init();
	tpm_init();

	prev_state = next_state = STATE_IDLE;

	while(1)
	{
		state_task[next_state]();
	}

	return 0;
}


void task_sleep()
{
	prev_state = STATE_SLEEP;
}


void task_idle()
{
	if(prev_state != STATE_IDLE)
	{
#if DEBUG_PRINT
		// do state initialization task
		PRINTF("\r\nEntering STATE_IDLE");
		set_state_flag();
#endif
	}
	prev_state = STATE_IDLE;

	// check pushbuttons
	if(!GPIO_DRV_ReadPinInput(kGpioDI1))
	{
		next_state = STATE_PLAY;
	}
	else if(!GPIO_DRV_ReadPinInput(kGpioDI2))
	{
		next_state = STATE_RECORD;
	}
}


void task_play()
{
	uint8_t cmdBuffer[AUDIO_BUFSIZE+4];
	uint8_t datBuffer[10];
	static uint16_t dataSize = 0;

	// check entering condition
	if(prev_state != STATE_PLAY)
	{
		// read header of the EEPROM
		cmdBuffer[0] = RDDATA;
		cmdBuffer[1] = (uint8_t)(EEPROM_HEADER >> 16);
		cmdBuffer[2] = (uint8_t)(EEPROM_HEADER >> 8);
		cmdBuffer[3] = (uint8_t)(EEPROM_HEADER);
		//
		GPIO_DRV_ClearPinOutput(kGpioSS);
        SPI_DRV_MasterTransferBlocking(SPI_INSTANCE, NULL, cmdBuffer,
				datBuffer, 7, SPI_TIMEOUT);
		GPIO_DRV_SetPinOutput(kGpioSS);

		// get data size
		dataSize = 0;
		dataSize = datBuffer[4];
		dataSize = (dataSize << 8) + datBuffer[5];
		dataSize = (dataSize << 8) + datBuffer[6];
#if DEBUG_PRINT
		PRINTF("\r\ndatBuffer[4]: %d", datBuffer[4]);
		PRINTF("\r\ndatBuffer[5]: %d", datBuffer[5]);
		PRINTF("\r\ndatBuffer[6]: %d", datBuffer[6]);
		PRINTF("\r\nPlaying data size: %d", dataSize);
#endif
		// reset buffer address
		addrEEPROM = EEPROM_DATA;

		// fill the first buffer
		cmdBuffer[0] = RDDATA;
		// at address location
		cmdBuffer[1] = (uint8_t)(addrEEPROM >> 16);
		cmdBuffer[2] = (uint8_t)(addrEEPROM >> 8);
		cmdBuffer[3] = (uint8_t)(addrEEPROM);
		//
		GPIO_DRV_ClearPinOutput(kGpioSS);
		SPI_DRV_MasterTransferBlocking(SPI_INSTANCE, NULL, cmdBuffer,
				audioBuff1, AUDIO_BUFSIZE + 4, SPI_TIMEOUT);
		GPIO_DRV_SetPinOutput(kGpioSS);
		// FIXME: temporary 8bit to 12bit conversion
		for(int i = 4; i <AUDIO_BUFSIZE+4; i++)
		{
			audioBuff1X[i] = audioBuff1[i] << BUFF_SHIFT;
		}
		// set buffer index
		idxAudioBuff = 1;
		// advance buffer address
		addrEEPROM = addrEEPROM + AUDIO_BUFSIZE;
		// create a semaphore
		OSA_SemaCreate(&sema, 0);
		// change prev state
		prev_state = STATE_PLAY;

#if DEBUG_PRINT
		// set debug output
		PRINTF("\r\nEntering STATE_PLAY");
		set_state_flag();
#endif

		// start TPM for DAC conversion
		TPM_DRV_CounterStart(TPM_INSTSPK, kTpmCountingUp, TPM_FREQ, true);
	}
	else
	{
		// check exit condition
		if((addrEEPROM >= dataSize) || (addrEEPROM >= EEPROM_MAXADDR))
		{
			// stop recording
			TPM_DRV_CounterStop(TPM_INSTSPK);
			// destroy semaphore
			OSA_SemaDestroy(&sema);
			// return to the idle state
			next_state = STATE_IDLE;
#if DEBUG_PRINT
			// print the last buffer for verification
			PRINTF("\r\nBuffer 1:");
			for(int i = 0; i < AUDIO_BUFSIZE; i++)
				PRINTF("%d.",audioBuff1[i+4]);
			PRINTF("\r\nBuffer 2:");
			for(int i = 0; i < AUDIO_BUFSIZE; i++)
				PRINTF("%d.",audioBuff2[i+4]);
#endif
		}
		// read data
		else if(kStatus_OSA_Success == OSA_SemaWait(&sema, 0))
		{
			// read EEPROM DATA
			cmdBuffer[0] = RDDATA;
			// at address location
			cmdBuffer[1] = (uint8_t)(addrEEPROM >> 16);
			cmdBuffer[2] = (uint8_t)(addrEEPROM >> 8);
			cmdBuffer[3] = (uint8_t)(addrEEPROM);

			// fill buffer with data
			if(idxAudioBuff == 1)
			{
				GPIO_DRV_ClearPinOutput(kGpioSS);
				SPI_DRV_MasterTransferBlocking(SPI_INSTANCE, NULL, cmdBuffer,
						audioBuff2, AUDIO_BUFSIZE + 4, SPI_TIMEOUT);
				GPIO_DRV_SetPinOutput(kGpioSS);
				// FIXME: temporary 8bit to 12bit conversion
				for(int i = 4; i <AUDIO_BUFSIZE+4; i++)
				{
					audioBuff2X[i] = audioBuff2[i] << BUFF_SHIFT;
				}
			}
			else
			{
				GPIO_DRV_ClearPinOutput(kGpioSS);
				SPI_DRV_MasterTransferBlocking(SPI_INSTANCE, NULL, cmdBuffer,
						audioBuff1, AUDIO_BUFSIZE + 4, SPI_TIMEOUT);
				GPIO_DRV_SetPinOutput(kGpioSS);
				// FIXME: temporary 8bit to 12bit conversion
				for(int i = 4; i <AUDIO_BUFSIZE+4; i++)
				{
					audioBuff1X[i] = audioBuff1[i] << BUFF_SHIFT;
				}
			}
			// update buffer address
			addrEEPROM = addrEEPROM + AUDIO_BUFSIZE;
		}
	}
}


/*
 * Audio recording task
 *
 * TPM -> ADC -> DMA
 */
void task_record()
{
	uint8_t cmdBuffer[10];

	// check entering condition
	if(prev_state != STATE_RECORD)
	{
		// initialize EEPROM address
		addrEEPROM = EEPROM_DATA;
		// initialize buffer index
		idxAudioBuff = 1;
		// create a semaphore
		OSA_SemaCreate(&sema, 0);
		// change prev_state
		prev_state = STATE_RECORD;
		// start TPM for ADC conversion
		TPM_DRV_CounterStart(TPM_INSTMIC, kTpmCountingUp, TPM_FREQ, true);

#if DEBUG_PRINT
		// set debug output
		PRINTF("\r\nEntering STATE_RECORD");
		set_state_flag();
#endif
	}
	else
	{
		// check exit conditions first
		if(GPIO_DRV_ReadPinInput(kGpioDI2) || (addrEEPROM>=EEPROM_MAXADDR))
		{
			// stop recording
			TPM_DRV_CounterStop(TPM_INSTMIC);
			// destroy semaphore
			OSA_SemaDestroy(&sema);
			// short delay
			OSA_TimeDelay(10u);

			// set EEPROM write enable latch
			cmdBuffer[0] = SETWEL;
			GPIO_DRV_ClearPinOutput(kGpioSS);
			SPI_DRV_MasterTransferBlocking(SPI_INSTANCE, NULL, cmdBuffer,
					NULL, 1, SPI_TIMEOUT);
			GPIO_DRV_SetPinOutput(kGpioSS);
			// short time delay
			OSA_TimeDelay(1u);

			// save the size information on EEPROM
			cmdBuffer[0] = WRDATA;
			// at the header location
			cmdBuffer[1] = (uint8_t)(EEPROM_HEADER >> 16);
			cmdBuffer[2] = (uint8_t)(EEPROM_HEADER >> 8);
			cmdBuffer[3] = (uint8_t)(EEPROM_HEADER);
			// in three bytes
			cmdBuffer[4] = (uint8_t)(addrEEPROM >> 16);
			cmdBuffer[5] = (uint8_t)(addrEEPROM >> 8);
			cmdBuffer[6] = (uint8_t)(addrEEPROM);
			//
			GPIO_DRV_ClearPinOutput(kGpioSS);
			SPI_DRV_MasterTransferBlocking(SPI_INSTANCE, NULL, cmdBuffer,
					NULL, 7, SPI_TIMEOUT);
			GPIO_DRV_SetPinOutput(kGpioSS);

			// return to the idle state
			next_state = STATE_IDLE;

#if DEBUG_PRINT
			PRINTF("\r\ncmdBuffer[4]: %d", cmdBuffer[4]);
			PRINTF("\r\ncmdBuffer[5]: %d", cmdBuffer[5]);
			PRINTF("\r\ncmdBuffer[6]: %d", cmdBuffer[6]);
			PRINTF("\r\nRecording data size: %d", addrEEPROM);
#endif
		}
		// check semaphone
		else if(kStatus_OSA_Success == OSA_SemaWait(&sema, 0))
		{
			// set EEPROM write enable latch
			cmdBuffer[0] = SETWEL;
			GPIO_DRV_ClearPinOutput(kGpioSS);
			SPI_DRV_MasterTransferBlocking(SPI_INSTANCE, NULL, cmdBuffer,
					NULL, 1, SPI_TIMEOUT);
			GPIO_DRV_SetPinOutput(kGpioSS);
			// short time delay
			OSA_TimeDelay(1u);

			if(idxAudioBuff == 1)
			{
				// write buffer 2 data to EEPROM
				audioBuff2[0] = WRDATA;
				// at address location
				audioBuff2[1] = (uint8_t)(addrEEPROM >> 16);
				audioBuff2[2] = (uint8_t)(addrEEPROM >> 8);
				audioBuff2[3] = (uint8_t)(addrEEPROM);
				//
				GPIO_DRV_ClearPinOutput(kGpioSS);
				SPI_DRV_MasterTransferBlocking(SPI_INSTANCE, NULL, audioBuff2,
						NULL, AUDIO_BUFSIZE + 4, SPI_TIMEOUT);
				GPIO_DRV_SetPinOutput(kGpioSS);
			}
			else
			{
				// write buffer 1 data to EEPROM
				audioBuff1[0] = WRDATA;
				// at address location
				audioBuff1[1] = (uint8_t)(addrEEPROM >> 16);
				audioBuff1[2] = (uint8_t)(addrEEPROM >> 8);
				audioBuff1[3] = (uint8_t)(addrEEPROM);
				//
				GPIO_DRV_ClearPinOutput(kGpioSS);
				SPI_DRV_MasterTransferBlocking(SPI_INSTANCE, NULL, audioBuff1,
						NULL, AUDIO_BUFSIZE + 4, SPI_TIMEOUT);
				GPIO_DRV_SetPinOutput(kGpioSS);
			}
			// update address
			addrEEPROM = addrEEPROM + AUDIO_BUFSIZE;
		}
	}
}


/*
 * Set the GPIO DO pins - Debug purpose only
 */
void set_state_flag()
{
	if(next_state == STATE_SLEEP)
	{
		GPIO_DRV_SetPinOutput(kGpioDO1);
		GPIO_DRV_ClearPinOutput(kGpioDO2);
	}
	else if(next_state == STATE_PLAY)
	{
		GPIO_DRV_ClearPinOutput(kGpioDO1);
		GPIO_DRV_SetPinOutput(kGpioDO2);
	}
	else if(next_state == STATE_RECORD)
	{
		GPIO_DRV_SetPinOutput(kGpioDO1);
		GPIO_DRV_SetPinOutput(kGpioDO2);
	}
	else
	{
		GPIO_DRV_ClearPinOutput(kGpioDO1);
		GPIO_DRV_ClearPinOutput(kGpioDO2);
	}
}


/*
 * Initialize ADC module with
 *	- resolution 8 bit, single ended
 *	- hardware triggered (TPM)
 *	- invoke DMA request at the completion of conversion
 *
 */
void adc_init()
{
	// ADC0_SE8: PTB0 (this is redundant as it is default state)
	CLOCK_SYS_EnablePortClock(PORTB_IDX);
	PORT_HAL_SetMuxMode(PORTB,0u,kPortPinDisabled);

	// initialize the module with user parameter
	adc16_converter_config_t adcUserConfig =
	{
		.lowPowerEnable = false,							// ADCx_CFG1(ADLPC)
		.clkDividerMode = kAdc16ClkDividerOf1,				// ADCx_CFG1(ADIV)
		.longSampleTimeEnable = true,						// ADCx_CFG1(ADLSMP)
		/*
		 * NOTE THAT 16BIT RESOLUTION SETTING WITH HARDWARE TRIGGER YIELDS
		 * FAULTY BEHAVIOUR OF THE ADC MODULE.
		 * IT IS NOT CERTAIN THIS IS A SILICON ERROR OR NOT.
		 */
		.resolution = kAdc16ResolutionBitOf8or9,			// ADCx_CFG1(MODE)
		.clkSrc = kAdc16ClkSrcOfAsynClk,					// ADCx_CFG1(ADICLK)
		.asyncClkEnable = true,								// ADCx_CFG2(ADACKEN)
		.highSpeedEnable = false,							// ADCx_CFG2(ADHSC)
		.longSampleCycleMode = kAdc16LongSampleCycleOf6,	// ADCx_CFG2(ADLSTS)
		.hwTriggerEnable = true,							// ADCx_SC2(ADTRG)
		.refVoltSrc = kAdc16RefVoltSrcOfVref,				// ADCx_SC2(REFSEL)
		.continuousConvEnable = false,						// ADCx_SC3(ADC0)
		.dmaEnable = true									// ADCx_SC2(DMAEN)
	};
    ADC16_DRV_Init(ADC_INSTANCE, &adcUserConfig);

    // perform auto calibration
    adc16_calibration_param_t adcCalibraitionParam;
    ADC16_DRV_GetAutoCalibrationParam(ADC_INSTANCE, &adcCalibraitionParam);
    ADC16_DRV_SetCalibrationParam(ADC_INSTANCE, &adcCalibraitionParam);

    // Use hardware average to increase stability of the measurement.
    adc16_hw_average_config_t userHwAverageConfig = {
		.hwAverageEnable = true,						// ADCx_SC3(AVGE)
		.hwAverageCountMode = kAdc16HwAverageCountOf4	// ADCx_SC3(AVGS)
	};
    ADC16_DRV_ConfigHwAverage(ADC_INSTANCE, &userHwAverageConfig);

	// channel configuration
    adc16_chn_config_t chnConfig = {	// channel configuration
		.chnIdx = (adc16_chn_t)ADC_CHNMIC,			// ADCx_SC1n(ADCH)
		.diffConvEnable = false,					// ADCx_SC1n(DIFF)
		.convCompletedIntEnable = false				// ADCx_SC1n(AIEN)
	};

    // Configuration triggers the conversion in software conversion case
	// but not here.
    ADC16_DRV_ConfigConvChn(ADC_INSTANCE, ADC_CHNGROUP, &chnConfig);
}

/*
 * ADC IRQ handler
void ADC0_IRQHandler(void)
{
	// COCO flag check is unnecessary for this case
	//if(ADC16_DRV_GetChnFlag(ADC_INSTANCE, ADC_CHNGROUP, kAdcChnConvCompleteFlag))
	{
		adcValue = ADC16_DRV_GetConvValueRAW(ADC_INSTANCE, ADC_CHNGROUP);
		adcIsrFlag = true;
		GPIO_DRV_TogglePinOutput(kGpioDO1);
	}
}
 */


/*
 * Initialize DAC module
 */
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


/*
 * Initialize DMA module
 */
void dma_init()
{
	// initialize DMA module
    DMA_DRV_Init(&dmaState);

	/*
	*/
	// -------------------------------------------------------------------------
	// request a channel for use with ADC0(conversion complete)
	DMA_DRV_RequestChannel(DMA_CHNMIC, kDmaRequestMux0ADC0, &dmaChnMic);

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
	DMA_DRV_ConfigTransfer(&dmaChnMic, kDmaPeripheralToMemory, 1,
			(uint32_t)(&ADC0_RA), (uint32_t)audioBuff1, AUDIO_BUFSIZE);

	// keep running after done
	DMA_HAL_SetDisableRequestAfterDoneCmd(DMA_MODULE, DMA_CHNMIC, false);
	// register dma callback
	DMA_DRV_RegisterCallback(&dmaChnMic, DMA_CB_Mic, NULL);
	// start channel
	DMA_DRV_StartChannel(&dmaChnMic);

	// -------------------------------------------------------------------------
	// request a channel for use with TPM1(OVF trigger)
	DMA_DRV_RequestChannel(DMA_CHNSPK, kDmaRequestMux0TPM1Overflow, &dmaChnSpk);

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
	// FIXME
	DMA_DRV_ConfigTransfer(&dmaChnSpk, kDmaMemoryToPeripheral, 2,
			(uint32_t)audioBuff1X, (uint32_t)(&DAC0_DATL(0)), AUDIO_BUFSIZE*2);
	//DMA_DRV_ConfigTransfer(&dmaChnSpk, kDmaMemoryToPeripheral, 1,
	//		(uint32_t)audioBuff1, (uint32_t)(&DAC0_DATL(0)), AUDIO_BUFSIZE);

	// keep running after done
	DMA_HAL_SetDisableRequestAfterDoneCmd(DMA_MODULE, DMA_CHNSPK, false);
	// register dma callback
	DMA_DRV_RegisterCallback(&dmaChnSpk, DMA_CB_Spk, NULL);
	// start channel
	DMA_DRV_StartChannel(&dmaChnSpk);
}

/*
 * End of DMA transfer callback
 */
void DMA_CB_Mic(void *param, dma_channel_status_t chanStatus)
{
	// clear DMA status
	DMA_HAL_ClearStatus(DMA_MODULE, DMA_CHNMIC);
	// toggle buffer
	if(idxAudioBuff == 1)
	{
		DMA_HAL_SetDestAddr(DMA_MODULE, DMA_CHNMIC, (uint32_t)(audioBuff2+4));
		idxAudioBuff = 2;
	}
	else
	{
		DMA_HAL_SetDestAddr(DMA_MODULE, DMA_CHNMIC, (uint32_t)(audioBuff1+4));
		idxAudioBuff = 1;
	}
	// reset count
	DMA_BWR_DSR_BCR_BCR(DMA_MODULE, DMA_CHNMIC, AUDIO_BUFSIZE);

	// raise semaphone
	OSA_SemaPost(&sema);
}

/*
 * End of DMA transfer callback
 */
void DMA_CB_Spk(void *param, dma_channel_status_t chanStatus)
{
	GPIO_DRV_TogglePinOutput(kGpioDO2);
	// clear DMA status
	DMA_HAL_ClearStatus(DMA_MODULE, DMA_CHNSPK);
	// toggle buffer
	if(idxAudioBuff == 1)
	{
		// FIXME
		//DMA_HAL_SetSourceAddr(DMA_MODULE, DMA_CHNSPK, (uint32_t)(audioBuff2+4));
		DMA_HAL_SetSourceAddr(DMA_MODULE, DMA_CHNSPK, (uint32_t)(audioBuff2X+4));
		idxAudioBuff = 2;
	}
	else
	{
		// FIXME
		//DMA_HAL_SetSourceAddr(DMA_MODULE, DMA_CHNSPK, (uint32_t)(audioBuff1+4));
		DMA_HAL_SetSourceAddr(DMA_MODULE, DMA_CHNSPK, (uint32_t)(audioBuff1X+4));
		idxAudioBuff = 1;
	}
	// reset count
	// FIXME
	//DMA_BWR_DSR_BCR_BCR(DMA_MODULE, DMA_CHNSPK, AUDIO_BUFSIZE);
	DMA_BWR_DSR_BCR_BCR(DMA_MODULE, DMA_CHNSPK, AUDIO_BUFSIZE*2);

	// raise semaphone
	OSA_SemaPost(&sema);
}

/*
 * DMA IRQ handler for channel 0
 */
void DMA0_IRQHandler(void)
{
    DMA_DRV_IRQhandler(0);
}

/*
 * DMA IRQ handler for channel 1
 * Be ware the weird naming convention. DMA1 here means channel 1 of DMA0
 */
void DMA1_IRQHandler(void)
{
    DMA_DRV_IRQhandler(1);
}


/*
 * Initialize GPIO
 */
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


/*
 * Initialize I2C
 */
void i2c_init()
{
	CLOCK_SYS_EnablePortClock(PORTB_IDX);

	// I2C0: PTB2 and PTB3 as ALT2
	PORT_HAL_SetMuxMode(PORTB,2u,kPortMuxAlt2); // SCL
	PORT_HAL_SetMuxMode(PORTB,3u,kPortMuxAlt2); // SDA

	// OSA layer should be initialized first
	OSA_Init();

    // Initialize i2c master
    I2C_DRV_MasterInit(I2C_INSTANCE, &master);

	// program TPA2016 gain
	uint8_t reg_addr;
	uint8_t reg_val;

	reg_addr = REG_GAIN;
	reg_val = 0x1E;
	//reg_val = 0x06;
	I2C_DRV_MasterSendDataBlocking(I2C_INSTANCE, &tpa2016,
			&reg_addr, 1, &reg_val, 1, 1000);

	I2C_DRV_MasterReceiveDataBlocking(I2C_INSTANCE, &tpa2016,
			&reg_addr, 1, &reg_val, 1, 1000);

#if DEBUG_PRINT
	PRINTF("\r\nTPA2016 Gain Register : %02X", reg_val);
#endif

	reg_addr = REG_AGCG;
	reg_val = 0xC3;
	//reg_val = 0xC1;
	I2C_DRV_MasterSendDataBlocking(I2C_INSTANCE, &tpa2016,
			&reg_addr, 1, &reg_val, 1, 1000);

	I2C_DRV_MasterReceiveDataBlocking(I2C_INSTANCE, &tpa2016,
			&reg_addr, 1, &reg_val, 1, 1000);

#if DEBUG_PRINT
	PRINTF("\r\nTPA2016 AGC Register : %02X", reg_val);
#endif
}

/*
 * I2C IRQ handler
 */
void I2C0_IRQHandler(void)
{
    I2C_DRV_IRQHandler(I2C0_IDX);
}


/*
 * Initialize SPI module
 */
void spi_init(void)
{
	CLOCK_SYS_EnablePortClock(PORTC_IDX);

	// SPI port: SPI0 on C4..7 as ALT2
	PORT_HAL_SetMuxMode(PORTC,5u,kPortMuxAlt2); // SCK
	PORT_HAL_SetMuxMode(PORTC,6u,kPortMuxAlt2); // MOSI
	PORT_HAL_SetMuxMode(PORTC,7u,kPortMuxAlt2); // MISO

	// Note that the EEPROM protocol is not compatible to standard SPI
	// in such way that the SS signal should be held low for entire
	// read or write cycle otherwise the EEPROM will start a new
	// command cycle at each byte or word transfer.
	// Thus SS must be controlled manually with GPIO
	//PORT_HAL_SetMuxMode(PORTC,4u,kPortMuxAlt2); // PCS0
	PORT_HAL_SetMuxMode(PORTC,4u,kPortMuxAsGpio); // PCS0
	gpio_output_pin_user_config_t output_pin[] = {
		{
			.pinName = kGpioSS,
			.config.outputLogic = 0,
			.config.slewRate = kPortFastSlewRate,
			.config.driveStrength = kPortHighDriveStrength,
		},
		{
			.pinName = GPIO_PINS_OUT_OF_RANGE,
		}
	};
	GPIO_DRV_Init(NULL, output_pin);
	GPIO_DRV_SetPinOutput(kGpioSS);

    uint32_t calculatedBaudRate;
    spi_master_user_config_t userConfig =
    {
        .bitCount       = kSpi8BitMode,
        .polarity       = kSpiClockPolarity_ActiveHigh,
        .phase          = kSpiClockPhase_FirstEdge,
        .direction      = kSpiMsbFirst,
        .bitsPerSec     = SPI_BAUD
    };

	// OSA layer should be initialized first
	OSA_Init();

	// SPI master
    SPI_DRV_MasterInit(SPI_INSTANCE, &spiMasterState);
    SPI_DRV_MasterConfigureBus(SPI_INSTANCE, &userConfig, &calculatedBaudRate);

#if DEBUG_PRINT
    // Check if the configuration is correct
    if (calculatedBaudRate > userConfig.bitsPerSec)
    {
        PRINTF("\r**Something failed in the master bus config \r\n");
    }
    else
    {
        PRINTF("\r\nBaud rate in Hz is: %d\r\n", calculatedBaudRate);
    }
#endif
}

/*
 * SPI IRQ handler
 */
void SPI0_IRQHandler(void)
{
   SPI_DRV_IRQHandler(SPI0_IDX);
}


/*
 * Initialize TPM module
 */
void tpm_init()
{
	// TPM general config
	tpm_general_config_t tpm_cfg =
	{
		.isDBGMode = false,
		.isGlobalTimeBase = false,
		.isTriggerMode = false,
		.isStopCountOnOveflow = false,
		.isCountReloadOnTrig = true,
		.triggerSource = kTpmTrigSel0
	};

	// TPM instance 0 for ADC recording ----------------------------------------
	// tpm module clock selection: SIM_SOPT2(PLLFLLSEL)
	CLOCK_SYS_SetTpmSrc(TPM_INSTMIC, kClockTpmSrcPllFllSel);
	// enable tpm module clock: this is redundant
	//CLOCK_SYS_EnableTpmClock(TPM_INSTMIC);
	/*
	*/
	// tpm module initialization
	TPM_DRV_Init(TPM_INSTMIC, &tpm_cfg);
	// tpm clock selection: SIM_SOPT2(TPMSRC): 48MHz
	TPM_DRV_SetClock(TPM_INSTMIC, kTpmClockSourceModuleClk, TPM_DIV);
	// do not start counter yet

	// configure SIM for ADC hw trigger source: SIM_SOPT7
	SIM_HAL_SetAdcAlternativeTriggerCmd(SIM, ADC_INSTANCE, true);
	// Trigger Selection A corresponds to channel group 0(SC1A)
	SIM_HAL_SetAdcPreTriggerMode(SIM, ADC_INSTANCE, kSimAdcPretrgselA);
	SIM_HAL_SetAdcTriggerMode(SIM, ADC_INSTANCE, kSimAdcTrgSelTpm0);

	// TPM instance 1 for DAC playing ------------------------------------------
	// tpm module clock selection: SIM_SOPT2(PLLFLLSEL)
	CLOCK_SYS_SetTpmSrc(TPM_INSTSPK, kClockTpmSrcPllFllSel);
	// enable tpm module clock: this is redundant
	//CLOCK_SYS_EnableTpmClock(TPM_INSTSPK);

	// tpm module initialization
	TPM_DRV_Init(TPM_INSTSPK, &tpm_cfg);
	// tpm clock selection: SIM_SOPT2(TPMSRC): 48MHz
	TPM_DRV_SetClock(TPM_INSTSPK, kTpmClockSourceModuleClk, TPM_DIV);
	//
	// enable DMA trigger
	//
	// Note that this feature is not exposed via HAL nor Peripheral Library
	// as of this writing. Thus direct call is necessary here.
	//
	TPM_WR_SC_DMA(TPM_MODSPK, 1);
	// do not start counter yet
}

/*
 * TPM0 IRQ handler
 */
void TPM0_IRQHandler(void)
{
	TPM_HAL_ClearTimerOverflowFlag(TPM_MODMIC);
}

/*
 * TPM1 IRQ handler
 * FIXME: this is not necessary.
 */
void TPM1_IRQHandler(void)
{
	//GPIO_DRV_TogglePinOutput(kGpioDO1);
	TPM_HAL_ClearTimerOverflowFlag(TPM_MODSPK);
}

// end of file
