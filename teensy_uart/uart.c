#include "teensy_lc.h"
#include "fsl_gpio_driver.h"
#include "fsl_uart_driver.h"

#define UART_BAUD		115200	// choose valid baud
#define UART_INST		1		// either 1 or 2

enum _gpio_pins
{
	kGpioPBtn1 = GPIO_MAKE_PIN(PORTC_IDX, 7u),
	kGpioLED1 = GPIO_MAKE_PIN(PORTC_IDX, 5u),
};

uart_state_t ua_state;
const uint8_t msg1[] = "\r\nUART driver initialized.";

extern void UART_DRV_IRQHandler(uint32_t inst);
void gpio_init(void);
void uart_init(uint32_t inst, uint32_t baud);

int main(void)
{
	uint8_t rxChar;

	board_init();
	gpio_init();
	uart_init(UART_INST, UART_BAUD);

	GPIO_DRV_SetPinOutput(kGpioLED1);

	if(kStatus_UART_Success == UART_DRV_SendData(1, msg1, sizeof(msg1)))
	{
		// wait until tx ends
		while(kStatus_UART_TxBusy ==
				UART_DRV_GetTransmitStatus(UART_INST, NULL)){};
	}

	GPIO_DRV_ClearPinOutput(kGpioLED1);

	while(1)
	{
		// loopback test
		if(kStatus_UART_Success ==
				UART_DRV_ReceiveData(UART_INST, &rxChar, 1))
		{
			// wait until rx ends
			while(kStatus_UART_RxBusy ==
					UART_DRV_GetReceiveStatus(UART_INST, NULL)){};
			// echo back
			UART_DRV_SendData(UART_INST, &rxChar, 1);
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

void uart_init(uint32_t inst, uint32_t baud)
{
	if(inst == 1)
	{
		// enable port C clock
		CLOCK_SYS_EnablePortClock(PORTC_IDX);	// SIM_SCGC5(PORTC)
		// UART0 port initialization
		PORT_HAL_SetMuxMode(PORTC,3,kPortMuxAlt3);	// UART1_RX
		PORT_HAL_SetMuxMode(PORTC,4,kPortMuxAlt3);	// UART1_TX
	}
	else if(inst == 2)
	{
		// enable port D clock
		CLOCK_SYS_EnablePortClock(PORTD_IDX);	// SIM_SCGC5(PORTD)
		// UART0 port initialization
		PORT_HAL_SetMuxMode(PORTD,2,kPortMuxAlt3);	// UART2_RX
		PORT_HAL_SetMuxMode(PORTD,3,kPortMuxAlt3);	// UART3_TX
	}
	else
	{
		return;
	}

	// enable module clock
	CLOCK_SYS_EnableUartClock(inst);

	// init driver
	uart_user_config_t ua_cfg = {
		.bitCountPerChar = kUart8BitsPerChar,
		.parityMode = kUartParityDisabled,
		.stopBitCount = kUartOneStopBit,
		.baudRate = baud
	};

	UART_DRV_Init(inst, &ua_state, &ua_cfg);
}

#if (UART_INST == 1)
/* Implementation of UART1 handler named in startup code. */
void UART1_IRQHandler(void)
{
    UART_DRV_IRQHandler(1);
}
#elif (UART_INST == 2)
/* Implementation of UART2 handler named in startup code. */
void UART2_IRQHandler(void)
{
    UART_DRV_IRQHandler(2);
}
#endif
