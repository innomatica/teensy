#include "teensy_lc.h"
#include "at25m.h"

#include "fsl_spi_master_driver.h"
#include "fsl_spi_shared_function.h"

// gpio
enum _gpio_pins
{
	kGpioSS = GPIO_MAKE_PIN(PORTC_IDX, 4u),
};
// spi
#define SPI_INSTANCE				(0)
#define TRANSFER_SIZE				(64)
#define TRANSFER_BAUDRATE			(500000U)
#define MASTER_TRANSFER_TIMEOUT		(5000U)

void spi_init(void);

uint8_t s_spiSinkBuffer[TRANSFER_SIZE] = {0};
uint8_t s_spiSourceBuffer[TRANSFER_SIZE] = {0};

spi_master_state_t spiMasterState;

int main(void)
{
	board_init();
	dbg_init(0, 115200);
	spi_init();

    while(1)
    {
		// enable write
		s_spiSourceBuffer[0] = SETWEL;

		GPIO_DRV_ClearPinOutput(kGpioSS);
        SPI_DRV_MasterTransferBlocking(SPI_INSTANCE, NULL, s_spiSourceBuffer,
				s_spiSinkBuffer, 1, MASTER_TRANSFER_TIMEOUT);
		GPIO_DRV_SetPinOutput(kGpioSS);

		// delay
		// check the chip select high time
        OSA_TimeDelay(1u);

		// read status
		s_spiSourceBuffer[0] = RDSREG;

		GPIO_DRV_ClearPinOutput(kGpioSS);
        SPI_DRV_MasterTransferBlocking(SPI_INSTANCE, NULL, s_spiSourceBuffer,
				s_spiSinkBuffer, 2, MASTER_TRANSFER_TIMEOUT);
		GPIO_DRV_SetPinOutput(kGpioSS);

		PRINTF("\r\nStatus Before Write: %02X", s_spiSinkBuffer[1]);

		// write 10 bytes of data
		s_spiSourceBuffer[0] = WRDATA;
		s_spiSourceBuffer[1] = 0x00;
		s_spiSourceBuffer[2] = 0x11;
		s_spiSourceBuffer[3] = 0x00;
		for(int i = 0; i < 10; i++)
		{
			s_spiSourceBuffer[i+4] = i;
		}

		GPIO_DRV_ClearPinOutput(kGpioSS);
        SPI_DRV_MasterTransferBlocking(SPI_INSTANCE, NULL, s_spiSourceBuffer,
				NULL, 14, MASTER_TRANSFER_TIMEOUT);
		GPIO_DRV_SetPinOutput(kGpioSS);

		// this delay seems to be critical
		// check the write cycle time of the device
        OSA_TimeDelay(10u);

		// read status again
		s_spiSourceBuffer[0] = RDSREG;

		GPIO_DRV_ClearPinOutput(kGpioSS);
        SPI_DRV_MasterTransferBlocking(SPI_INSTANCE, NULL, s_spiSourceBuffer,
				s_spiSinkBuffer, 2, MASTER_TRANSFER_TIMEOUT);
		GPIO_DRV_SetPinOutput(kGpioSS);

		PRINTF("\r\nStatus After Write: %02X", s_spiSinkBuffer[1]);

		// read data back
		s_spiSourceBuffer[0] = RDDATA;
		s_spiSourceBuffer[1] = 0x00;
		s_spiSourceBuffer[2] = 0x11;
		s_spiSourceBuffer[3] = 0x00;

		GPIO_DRV_ClearPinOutput(kGpioSS);
        SPI_DRV_MasterTransferBlocking(SPI_INSTANCE, NULL, s_spiSourceBuffer,
				s_spiSinkBuffer, 14, MASTER_TRANSFER_TIMEOUT);
		GPIO_DRV_SetPinOutput(kGpioSS);

		PRINTF("\r\nData: ");

		for(int i = 0; i < 10; i++)
		{
			PRINTF("%02X.", s_spiSinkBuffer[i+4]);
		}


        // Wait for press any key.
        PRINTF("\r\nPress any key to run again\r\n");
        GETCHAR();
    }
	return 0;
}


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
        .bitsPerSec     = TRANSFER_BAUDRATE
    };

	// OSA layer should be initialized first
	OSA_Init();

	// SPI master
    SPI_DRV_MasterInit(SPI_INSTANCE, &spiMasterState);
    SPI_DRV_MasterConfigureBus(SPI_INSTANCE, &userConfig, &calculatedBaudRate);

    // Check if the configuration is correct
    if (calculatedBaudRate > userConfig.bitsPerSec)
    {
        PRINTF("\r**Something failed in the master bus config \r\n");
    }
    else
    {
        PRINTF("\r\nBaud rate in Hz is: %d\r\n", calculatedBaudRate);
    }
}


void SPI0_IRQHandler(void)
{
   SPI_DRV_IRQHandler(SPI0_IDX);
}
