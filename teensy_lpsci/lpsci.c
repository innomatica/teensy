#include "teensy_lc.h"
#include "fsl_gpio_driver.h"
#include "fsl_lpsci_driver.h"

#define LPSPI_BAUD	115200

enum _gpio_pins
{
	kGpioPBtn1 = GPIO_MAKE_PIN(PORTC_IDX, 7u),
	kGpioLED1 = GPIO_MAKE_PIN(PORTC_IDX, 5u),
};

lpsci_state_t lps_state;
const uint8_t msg1[] = "\r\nLPSCI driver initialized.";

extern void LPSCI_DRV_IRQHandler(uint32_t instance);
void gpio_init(void);
void lpspi_init(uint32_t baud);

int main(void)
{
	uint8_t rxChar;

	board_init();
	gpio_init();
	lpspi_init(LPSPI_BAUD);

	GPIO_DRV_SetPinOutput(kGpioLED1);

	if(kStatus_LPSCI_Success == LPSCI_DRV_SendData(0, msg1, sizeof(msg1)))
	{
		// wait until tx ends
		while(kStatus_LPSCI_TxBusy ==
				LPSCI_DRV_GetTransmitStatus(0, NULL)){};
	}

	GPIO_DRV_ClearPinOutput(kGpioLED1);

	while(1)
	{
		// loopback test
		if(kStatus_LPSCI_Success ==
				LPSCI_DRV_ReceiveData(0, &rxChar, 1))
		{
			// wait until rx ends
			while(kStatus_LPSCI_RxBusy ==
					LPSCI_DRV_GetReceiveStatus(0, NULL)){};
			// echo back
			LPSCI_DRV_SendData(0, &rxChar, 1);
		}
	}

	return 0;
}


void gpio_init(void)
{
	// enable port C clock
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


void lpspi_init(uint32_t baud)
{
	// enable port A clock
	CLOCK_SYS_EnablePortClock(PORTA_IDX);	// SIM_SCGC5(PORTA)
	// UART0 pin assignment
	PORT_HAL_SetMuxMode(PORTA,1,kPortMuxAlt2);
	PORT_HAL_SetMuxMode(PORTA,2,kPortMuxAlt2);

	lpsci_user_config_t lps_cfg = {
		.clockSource = kClockLpsciSrcPllFllSel,
		.bitCountPerChar = kLpsci8BitsPerChar,
		.parityMode = kLpsciParityDisabled,
		.stopBitCount = kLpsciOneStopBit,
		.baudRate = baud
	};
	// lpspi initialization
	LPSCI_DRV_Init(0, &lps_state, &lps_cfg);
}

/*
 * UART0 IRQ handler should be redirected to LPSCI_DRV_IRQHandler
 */
void UART0_IRQHandler(void)
{
    LPSCI_DRV_IRQHandler(0);
}

