#include "teensy_lc.h"
#include "tpa2016.h"

#include "fsl_i2c_master_driver.h"

#define I2C_INSTANCE			(0)
#define DATA_LENGTH             64

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

int main(void)
{
	board_init();
	dbg_init(0, 115200);
	i2c_init();
	PRINTF("\r\nI2C0 initialized.");


	uint8_t reg_addr;

	while(1)
	{
		reg_addr = REG_ICFC;
		// read registers
		I2C_DRV_MasterReceiveDataBlocking(I2C_INSTANCE, &tpa2016,
				&reg_addr, 1, (uint8_t*)txBuff, 7, 1000);

		PRINTF("\r\nRegister Value:");

		for(int i = 1; i < 8; i++)
		{
			PRINTF("%02X.",txBuff[i-1]);
		}

		GETCHAR();
	}


	return 0;
}


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
}

void I2C0_IRQHandler(void)
{
    I2C_DRV_IRQHandler(I2C0_IDX);
}
