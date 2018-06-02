/**HEADER********************************************************************
 *
 * Copyright (c) 2008, 2013 - 2015 Freescale Semiconductor;
 * All Rights Reserved
 *
 * Copyright (c) 1989-2008 ARC International;
 * All Rights Reserved
 *
 ***************************************************************************
 *
 * THIS SOFTWARE IS PROVIDED BY FREESCALE "AS IS" AND ANY EXPRESSED OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL FREESCALE OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
 **************************************************************************
 *
 * $FileName: virtual_com.c$
 * $Version :
 * $Date    :
 *
 * Comments:
 *
 * @brief  The file emulates a USB PORT as RS232 PORT.
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "teensy_lc.h"
#include "fsl_gpio_driver.h"
#include "fsl_pit_driver.h"
#include "fsl_adc16_driver.h"
#include "fsl_pmc_hal.h"
#include "usb_daq.h"
#include "usb_descriptor.h"

/*****************************************************************************
 * Constant and Macro's - None
 *****************************************************************************/

uint8_t usb_device_board_init(uint8_t controller_id);

/*****************************************************************************
 * Global Functions Prototypes
 *****************************************************************************/

/****************************************************************************
 * Global Variables
 ****************************************************************************/
extern usb_desc_request_notify_struct_t desc_callback;
extern uint8_t USB_Desc_Set_Speed(uint32_t handle, uint16_t speed);
cdc_handle_t g_app_handle;

/*****************************************************************************
 * Local Types - None
 *****************************************************************************/
enum _gpio_pins
{
	kGpioPBtn1 = GPIO_MAKE_PIN(PORTC_IDX, 7u),
	kGpioLED1 = GPIO_MAKE_PIN(PORTC_IDX, 5u),
};

#define ADC_INST			(0)		// ADC instacne
#define ADC_CHAN			(14)	// ADC channel
#define ADC_CGRP			(0)		// ADC channel group

void gpio_init(void);
void pit_init(void);
void adc_init(uint32_t adc_no);

/*****************************************************************************
 * Local Functions Prototypes
 *****************************************************************************/
void USB_App_Device_Callback(uint8_t event_type, void* val, void* arg);
uint8_t USB_App_Class_Callback(uint8_t event, uint16_t value, uint8_t ** data,
		uint32_t* size, void* arg);
void Virtual_Com_App(void);


/*****************************************************************************
 * Local Variables
 *****************************************************************************/
uint8_t g_line_coding[LINE_CODING_SIZE] =
{
	/*e.g. 0x00,0x10,0x0E,0x00 : 0x000E1000 is 921600 bits per second */
	(LINE_CODE_DTERATE_IFACE >> 0) & 0x000000FF,
	(LINE_CODE_DTERATE_IFACE >> 8) & 0x000000FF,
	(LINE_CODE_DTERATE_IFACE >> 16) & 0x000000FF,
	(LINE_CODE_DTERATE_IFACE >> 24) & 0x000000FF,
	LINE_CODE_CHARFORMAT_IFACE,
	LINE_CODE_PARITYTYPE_IFACE,
	LINE_CODE_DATABITS_IFACE
};

uint8_t g_abstract_state[COMM_FEATURE_DATA_SIZE] =
{
	(STATUS_ABSTRACT_STATE_IFACE >> 0) & 0x00FF,
	(STATUS_ABSTRACT_STATE_IFACE >> 8) & 0x00FF
};

uint8_t g_country_code[COMM_FEATURE_DATA_SIZE] =
{
	(COUNTRY_SETTING_IFACE >> 0) & 0x00FF,
	(COUNTRY_SETTING_IFACE >> 8) & 0x00FF
};

static bool start_app = FALSE;
static bool start_transactions = FALSE;

static uint8_t g_curr_recv_buf[DATA_BUFF_SIZE];
static uint8_t g_curr_send_buf[DATA_BUFF_SIZE];

static uint32_t g_recv_size;
static uint32_t g_send_size;

static uint16_t g_cdc_device_speed;
static uint16_t g_bulk_out_max_packet_size;
static uint16_t g_bulk_in_max_packet_size;

volatile uint32_t cmd_buf_head = 0;
volatile uint32_t cmd_buf_tail = 0;
volatile uint32_t dat_buf_head = 0;
volatile uint32_t dat_buf_tail = 0;
volatile int32_t cmd_count = 0;

volatile uint8_t cmd_buf[UDAQ_CMD_BUFSIZE];
volatile uint16_t dat_buf[UDAQ_DAT_BUFSIZE];
volatile bool adc_flag;
/*****************************************************************************
 * Local Functions
 *****************************************************************************/
void parse_and_run_cmd(void);
void capture_start(void);
void capture_stop(void);

/**************************************************************************//*!
 *
 * @name  USB_Get_Line_Coding
 *
 * @brief The function returns the Line Coding/Configuration
 *
 * @param handle:        handle
 * @param interface:     interface number
 * @param coding_data:   output line coding data
 *
 * @return USB_OK                              When Success
 *         USBERR_INVALID_REQ_TYPE             when Error
 *****************************************************************************/
uint8_t USB_Get_Line_Coding(uint32_t handle, uint8_t interface,
		uint8_t * *coding_data)
{
	//UNUSED_ARGUMENT(handle)
	/* if interface valid */
	if (interface < USB_MAX_SUPPORTED_INTERFACES)
	{
		/* get line coding data*/
		*coding_data = g_line_coding;
		return USB_OK;
	}

	return USBERR_INVALID_REQ_TYPE;
}

/**************************************************************************//*!
 *
 * @name  USB_Set_Line_Coding
 *
 * @brief The function sets the Line Coding/Configuration
 *
 * @param handle: handle
 * @param interface:     interface number
 * @param coding_data:   output line coding data
 *
 * @return USB_OK                              When Success
 *         USBERR_INVALID_REQ_TYPE             when Error
 *****************************************************************************/
uint8_t USB_Set_Line_Coding(uint32_t handle, uint8_t interface,
		uint8_t * *coding_data)
{
	uint8_t count;

	//UNUSED_ARGUMENT(handle)

	/* if interface valid */
	if (interface < USB_MAX_SUPPORTED_INTERFACES)
	{
		/* set line coding data*/
		for (count = 0; count < LINE_CODING_SIZE; count++)
		{
			g_line_coding[count] = *((*coding_data + USB_SETUP_PKT_SIZE) + count);
		}
		return USB_OK;
	}

	return USBERR_INVALID_REQ_TYPE;
}

/**************************************************************************//*!
 *
 * @name  USB_Get_Abstract_State
 *
 * @brief The function gets the current setting for communication feature
 *                                                  (ABSTRACT_STATE)
 * @param handle:        handle
 * @param interface:     interface number
 * @param feature_data:   output comm feature data
 *
 * @return USB_OK                              When Success
 *         USBERR_INVALID_REQ_TYPE             when Error
 *****************************************************************************/
uint8_t USB_Get_Abstract_State(uint32_t handle, uint8_t interface,
		uint8_t * *feature_data)
{
	//UNUSED_ARGUMENT(handle)
	/* if interface valid */
	if (interface < USB_MAX_SUPPORTED_INTERFACES)
	{
		/* get line coding data*/
		*feature_data = g_abstract_state;
		return USB_OK;
	}

	return USBERR_INVALID_REQ_TYPE;
}

/**************************************************************************//*!
 *
 * @name  USB_Get_Country_Setting
 *
 * @brief The function gets the current setting for communication feature
 *                                                  (COUNTRY_CODE)
 * @param handle:        handle
 * @param interface:     interface number
 * @param feature_data:   output comm feature data
 *
 * @return USB_OK                              When Success
 *         USBERR_INVALID_REQ_TYPE             when Error
 *****************************************************************************/
uint8_t USB_Get_Country_Setting(uint32_t handle, uint8_t interface,
		uint8_t * *feature_data)
{
	//UNUSED_ARGUMENT(handle)
	/* if interface valid */
	if (interface < USB_MAX_SUPPORTED_INTERFACES)
	{
		/* get line coding data*/
		*feature_data = g_country_code;
		return USB_OK;
	}

	return USBERR_INVALID_REQ_TYPE;
}

/**************************************************************************//*!
 *
 * @name  USB_Set_Abstract_State
 *
 * @brief The function gets the current setting for communication feature
 *                                                  (ABSTRACT_STATE)
 * @param handle:        handle
 * @param interface:     interface number
 * @param feature_data:   output comm feature data
 *
 * @return USB_OK                              When Success
 *         USBERR_INVALID_REQ_TYPE             when Error
 *****************************************************************************/
uint8_t USB_Set_Abstract_State(uint32_t handle, uint8_t interface,
		uint8_t * *feature_data)
{
	uint8_t count;
	//UNUSED_ARGUMENT(handle)
	/* if interface valid */
	if (interface < USB_MAX_SUPPORTED_INTERFACES)
	{
		/* set Abstract State Feature*/
		for (count = 0; count < COMM_FEATURE_DATA_SIZE; count++)
		{
			g_abstract_state[count] = *(*feature_data + count);
		}
		return USB_OK;
	}

	return USBERR_INVALID_REQ_TYPE;
}

/**************************************************************************//*!
 *
 * @name  USB_Set_Country_Setting
 *
 * @brief The function gets the current setting for communication feature
 *                                                  (COUNTRY_CODE)
 * @param handle: handle
 * @param interface:     interface number
 * @param feature_data:   output comm feature data
 *
 * @return USB_OK                              When Success
 *         USBERR_INVALID_REQ_TYPE             when Error
 *****************************************************************************/
uint8_t USB_Set_Country_Setting(uint32_t handle, uint8_t interface,
		uint8_t * *feature_data)
{
	uint8_t count;
	//UNUSED_ARGUMENT (handle)

	/* if interface valid */
	if (interface < USB_MAX_SUPPORTED_INTERFACES)
	{
		for (count = 0; count < COMM_FEATURE_DATA_SIZE; count++)
		{
			g_country_code[count] = *(*feature_data + count);
		}
		return USB_OK;
	}

	return USBERR_INVALID_REQ_TYPE;
}
/*****************************************************************************
 *
 *    @name         APP_init
 *
 *    @brief         This function do initialization for APP.
 *
 *    @param         None
 *
 *    @return       None
 **
 *****************************************************************************/
void APP_init(void)
{
	cdc_config_struct_t cdc_config;
	cdc_config.cdc_application_callback.callback = USB_App_Device_Callback;
	cdc_config.cdc_application_callback.arg = &g_app_handle;
	cdc_config.vendor_req_callback.callback = NULL;
	cdc_config.vendor_req_callback.arg = NULL;
	cdc_config.class_specific_callback.callback = USB_App_Class_Callback;
	cdc_config.class_specific_callback.arg = &g_app_handle;
	cdc_config.board_init_callback.callback = usb_device_board_init;
	cdc_config.board_init_callback.arg = CONTROLLER_ID;
	cdc_config.desc_callback_ptr = &desc_callback;
	/* Always happen in control endpoint hence hard coded in Class layer*/

	g_cdc_device_speed = USB_SPEED_FULL;
	g_bulk_out_max_packet_size = FS_DIC_BULK_OUT_ENDP_PACKET_SIZE;
	g_bulk_in_max_packet_size = FS_DIC_BULK_IN_ENDP_PACKET_SIZE;

	/* Initialize the USB interface */
	USB_Class_CDC_Init(CONTROLLER_ID, &cdc_config, &g_app_handle);

	g_recv_size = 0;
	g_send_size = 0;

}


/******************************************************************************
 *
 *    @name       Virtual_Com_App
 *
 *    @brief
 *
 *    @param      None
 *
 *    @return     None
 *
 *****************************************************************************/
void Virtual_Com_App(void)
{
	// virtual_com_app activity indicator
	GPIO_DRV_TogglePinOutput(kGpioLED1);

    if ((0 != g_recv_size) && (0xFFFFFFFF != g_recv_size))
    {
		for (int i = 0; i < g_recv_size; i++)
		{
			cmd_buf[cmd_buf_head] = g_curr_recv_buf[i];
			if(cmd_buf[cmd_buf_head] == UDAQ_CMD_ENDMARK)
			{
				cmd_count++;
			}
			UDAQ_CMD_BUFPINC(cmd_buf_head);
		}

		g_recv_size = 0;

		// invoke next read cycle
		USB_Class_CDC_Recv_Data(g_app_handle, DIC_BULK_OUT_ENDPOINT,
				g_curr_recv_buf, g_bulk_out_max_packet_size);
    }

	if(cmd_count)
	{
		parse_and_run_cmd();
	}


	if(dat_buf_tail != dat_buf_head)
	{
		uint16_t adc_data = 0;
		uint32_t index = 0;
		uint8_t error;

		//while(dat_buf_tail != dat_buf_head)
		{
			adc_data = dat_buf[dat_buf_tail];
			UDAQ_DAT_BUFPINC(dat_buf_tail);
			g_curr_send_buf[index++] = (uint8_t)(adc_data>>8);
			g_curr_send_buf[index++] = (uint8_t)(adc_data);
			g_curr_send_buf[index++] = (uint8_t)(UDAQ_DAT_ENDMARK>>8);
			g_curr_send_buf[index++] = (uint8_t)(UDAQ_DAT_ENDMARK);
		}

		error = USB_Class_CDC_Send_Data(g_app_handle, DIC_BULK_IN_ENDPOINT,
				g_curr_send_buf, index);

		if (error != USB_OK)
		{
			PRINTF("\r\nUSB_Send_Data failed");
		}
	}

	return;

}


/******************************************************************************
 *
 *    @name        USB_App_Device_Callback
 *
 *    @brief       This function handles the callback
 *
 *    @param       handle : handle to Identify the controller
 *    @param       event_type : value of the event
 *    @param       val : gives the configuration value
 *
 *    @return      None
 *
 *****************************************************************************/
void USB_App_Device_Callback(uint8_t event_type, void* val, void* arg)
{
	uint32_t handle;
	handle = *((uint32_t *) arg);

	if (event_type == USB_DEV_EVENT_BUS_RESET)
	{
		start_app = FALSE;
		if (USB_OK == USB_Class_CDC_Get_Speed(handle, &g_cdc_device_speed))
		{
			USB_Desc_Set_Speed(handle, g_cdc_device_speed);
			if (USB_SPEED_HIGH == g_cdc_device_speed)
			{
				g_bulk_out_max_packet_size = HS_DIC_BULK_OUT_ENDP_PACKET_SIZE;
				g_bulk_in_max_packet_size = HS_DIC_BULK_IN_ENDP_PACKET_SIZE;
			}
			else
			{
				g_bulk_out_max_packet_size = FS_DIC_BULK_OUT_ENDP_PACKET_SIZE;
				g_bulk_in_max_packet_size = FS_DIC_BULK_IN_ENDP_PACKET_SIZE;
			}
		}
	}
	else if (event_type == USB_DEV_EVENT_CONFIG_CHANGED)
	{
		/* Schedule buffer for receive */
		USB_Class_CDC_Recv_Data(handle, DIC_BULK_OUT_ENDPOINT, g_curr_recv_buf,
				g_bulk_out_max_packet_size);
		start_app = TRUE;
	}
	else if (event_type == USB_DEV_EVENT_ERROR)
	{
		/* add user code for error handling */
	}
	return;
}

/******************************************************************************
 *
 *    @name        USB_App_Class_Callback
 *
 *    @brief       This function handles the callback for Get/Set report req
 *
 *    @param       request  :  request type
 *    @param       value    :  give report type and id
 *    @param       data     :  pointer to the data
 *    @param       size     :  size of the transfer
 *
 *    @return      status
 *                  USB_OK  :  if successful
 *                  else return error
 *
 *****************************************************************************/
uint8_t USB_App_Class_Callback(uint8_t event, uint16_t value, uint8_t ** data,
		uint32_t* size, void* arg)
{
	cdc_handle_t handle;
	uint8_t error = USB_OK;
	handle = *((cdc_handle_t *) arg);

	switch(event)
	{
		case GET_LINE_CODING:
			error = USB_Get_Line_Coding(handle, value, data);
			break;

		case GET_ABSTRACT_STATE:
			error = USB_Get_Abstract_State(handle, value, data);
			break;

		case GET_COUNTRY_SETTING:
			error = USB_Get_Country_Setting(handle, value, data);
			break;

		case SET_LINE_CODING:
			error = USB_Set_Line_Coding(handle, value, data);
			break;

		case SET_ABSTRACT_STATE:
			error = USB_Set_Abstract_State(handle, value, data);
			break;

		case SET_COUNTRY_SETTING:
			error = USB_Set_Country_Setting(handle, value, data);
			break;

		case USB_APP_CDC_DTE_ACTIVATED:
			capture_start();

			if (start_app == TRUE)
			{
				start_transactions = TRUE;
			}
			break;

		case USB_APP_CDC_DTE_DEACTIVATED:
			capture_stop();

			if (start_app == TRUE)
			{
				start_transactions = FALSE;
			}
			break;

		case USB_DEV_EVENT_DATA_RECEIVED:

			if ((start_app == TRUE) && (start_transactions == TRUE))
			{
				g_recv_size = *size;

				if (!g_recv_size)
				{
					/* Schedule buffer for receive */
					USB_Class_CDC_Recv_Data(handle, DIC_BULK_OUT_ENDPOINT,
							g_curr_recv_buf, g_bulk_out_max_packet_size);
				}
			}
			break;

		case USB_DEV_EVENT_SEND_COMPLETE:

			if ((size != NULL) && (*size != 0) &&
					(!(*size % g_bulk_in_max_packet_size)))
			{
				/* If the last packet is the size of endpoint, then send
				 * also zero-ended packet, meaning that we want to inform
				 * the host that we do not have any additional data,
				 * so it can flush the output.
				 */
				USB_Class_CDC_Send_Data(g_app_handle, DIC_BULK_IN_ENDPOINT,
						NULL, 0);
			}
			else if ((start_app == TRUE) && (start_transactions == TRUE))
			{
				if ((*data != NULL) || ((*data == NULL) && (*size == 0)))
				{
					/* User: add your own code for send complete event */
				}
			}
			break;

		case USB_APP_CDC_SERIAL_STATE_NOTIF:
			/* User: add your own code for serial_state notify event */
			break;

		default:
			error = USBERR_INVALID_REQ_TYPE;
			break;

	}

	return error;
}


static void Task_Start(void *arg)
{
	/* call the periodic task function */
	USB_CDC_Periodic_Task();

	/*check whether enumeration is complete or not */
	if ((start_app == TRUE) && (start_transactions == TRUE))
	{
		Virtual_Com_App();
	}
}


int main(void)
{
	board_init();
	gpio_init();
	adc_init(ADC_INST);
	pit_init();
	dbg_init(0,115200);

	/* Select the clock source for the TPM counter */
	CLOCK_SYS_SetTpmSrc(0, kClockTpmSrcPllFllSel);

	OSA_Init();
	APP_init();
	OS_Task_create(Task_Start, NULL, 4L, 1000L, "task_start", NULL);
	OSA_Start();

	return 1;
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
		.periodUs = 1000000U
	};
	// initiallize PIT module
	PIT_DRV_Init(0, true);
	// initialize pit channel 0
	PIT_DRV_InitChannel(0,0, &pitCh0Config);

	// start PIT0
	//PIT_DRV_StartTimer(0,0);

	// configure SIM for ADC hw trigger source: SIM_SOPT7
	SIM_HAL_SetAdcAlternativeTriggerCmd(SIM, ADC_INST, true);
	// Trigger Selection A corresponds to channel group 0(SC1A)
	SIM_HAL_SetAdcPreTriggerMode(SIM, ADC_INST, kSimAdcPretrgselA);
	SIM_HAL_SetAdcTriggerMode(SIM, ADC_INST, kSimAdcTrgSelPit0);
}


uint8_t usb_device_board_init(uint8_t controller_id)
{
	int8_t ret = 0;

	if (0 == controller_id)
	{
		/* TO DO */
		/*add board initialization code if have*/
	}
	else
	{
		ret = 1;
	}

	return ret;

}

void parse_and_run_cmd()
{
	uint8_t cmd_pkt[UDAQ_CMD_PKTSIZE];
	uint8_t index = 0;

	while(cmd_buf_tail != cmd_buf_head)
	{
		// copy cmd_buf contents into the local storage
		cmd_pkt[index] = cmd_buf[cmd_buf_tail];
		UDAQ_CMD_BUFPINC(cmd_buf_tail);

		// end-of-cmd-string encountered
		if (cmd_pkt[index] == UDAQ_CMD_ENDMARK)
		{
			// run command here
			// single byte command for now
			if(cmd_pkt[0] == UDAQ_CMD_CAPSTRT)
				capture_start();
			else if(cmd_pkt[0] == UDAQ_CMD_CAPSTOP)
				capture_stop();

			// decrease com_count
			cmd_count--;
			return;
		}
		else
		{
			index++;
		}
	}
}


void capture_start(void)
{
	// start ADC by starting PIT
	PIT_DRV_StartTimer(0,0);
	PRINTF("\n\rCapture started.");
}

void capture_stop(void)
{
	// stop ADC by stopping PIT
	PIT_DRV_StopTimer(0,0);
	PRINTF("\n\rCapture stopped.");
}


/*
 * ADC IRQ Handler
 */
void ADC0_IRQHandler(void)
{
	// COCO flag check is unnecessary for this example
	if(ADC16_DRV_GetChnFlag(ADC_INST, ADC_CGRP, kAdcChnConvCompleteFlag))
	{
		// append adc data to data buffer
		dat_buf[dat_buf_head] = ADC16_DRV_GetConvValueRAW(ADC_INST, ADC_CGRP);
		UDAQ_DAT_BUFPINC(dat_buf_head);
	}
}


void USB0_IRQHandler(void)
{
	usb_khci_irq_handler();
}
