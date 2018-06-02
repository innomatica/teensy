#include "teensy_lc.h"
#include "fsl_gpio_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_spi_shared_function.h"

// spi
#define SPI_MASTER_INSTANCE			(0)
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

    uint8_t loopCount = 0;
    uint32_t j;
    uint32_t failCount = 0;

    while(1)
    {
        // Initialize the source buffer
        for (j = 0; j < TRANSFER_SIZE; j++)
        {
            s_spiSourceBuffer[j] = j+loopCount;
        }

        // Reset the sink buffer
        for (j = 0; j < TRANSFER_SIZE; j++)
        {
            s_spiSinkBuffer[j] = 0;
        }

		PRINTF("\r\nBuffer ready");
        // Start transfer data to slave
        if (SPI_DRV_MasterTransferBlocking(SPI_MASTER_INSTANCE, NULL,
					s_spiSourceBuffer, NULL, TRANSFER_SIZE,
					MASTER_TRANSFER_TIMEOUT) == kStatus_SPI_Timeout)
        {
            PRINTF("\r\n**Sync transfer timed-out \r\n");
        }
		PRINTF("\r\nMaster Trasfer done");

        // Delay sometime to wait slave receive and send back data
        OSA_TimeDelay(500U);

        // Start receive data from slave by transmit NULL bytes
        if (SPI_DRV_MasterTransferBlocking(SPI_MASTER_INSTANCE, NULL, NULL,
                             s_spiSinkBuffer, TRANSFER_SIZE, MASTER_TRANSFER_TIMEOUT) == kStatus_SPI_Timeout)
        {
            PRINTF("\r\n**Sync transfer timed-out \r\n");
        }

        // Verify the contents of the master sink buffer
        // refer to the slave driver for the expected data pattern
        failCount = 0; // reset failCount variable

        for (j = 0; j < TRANSFER_SIZE; j++)
        {
            if (s_spiSinkBuffer[j] != s_spiSourceBuffer[j])
            {
                 failCount++;
            }
        }

        // Print out transmit buffer.
        PRINTF("\r\nMaster transmit:");
        for (j = 0; j < TRANSFER_SIZE; j++)
        {
            // Print 16 numbers in a line.
            if ((j & 0x0F) == 0)
            {
                PRINTF("\r\n    ");
            }
            PRINTF(" %02X", s_spiSourceBuffer[j]);
        }
        // Print out receive buffer.
        PRINTF("\r\nMaster receive:");
        for (j = 0; j < TRANSFER_SIZE; j++)
        {
            // Print 16 numbers in a line.
            if ((j & 0x0F) == 0)
            {
                PRINTF("\r\n    ");
            }
            PRINTF(" %02X", s_spiSinkBuffer[j]);
        }

        if (failCount == 0)
        {
            PRINTF("\r\n Spi master transfer succeed! \r\n");
        }
        else
        {
            PRINTF("\r\n **failures detected in Spi master transfer! \r\n");
        }

        // Wait for press any key.
        PRINTF("\r\nPress any key to run again\r\n");
        GETCHAR();
        loopCount++;
    }
	return 0;
}


void spi_init(void)
{
	CLOCK_SYS_EnablePortClock(PORTC_IDX);

	// SPI port: SPI0 on C4..7 as ALT2
	PORT_HAL_SetMuxMode(PORTC,4u,kPortMuxAlt2); // PCS0
	PORT_HAL_SetMuxMode(PORTC,5u,kPortMuxAlt2); // SCK
	PORT_HAL_SetMuxMode(PORTC,6u,kPortMuxAlt2); // MOSI
	PORT_HAL_SetMuxMode(PORTC,7u,kPortMuxAlt2); // MISO

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
    SPI_DRV_MasterInit(SPI_MASTER_INSTANCE, &spiMasterState);
    SPI_DRV_MasterConfigureBus(SPI_MASTER_INSTANCE,
                                &userConfig,
                                &calculatedBaudRate);

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
