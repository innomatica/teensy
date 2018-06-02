#ifndef __TEENSY_LC_H__
#define __TEENSY_LC_H__

#include <stdint.h>

#include "fsl_debug_console.h"
#include "fsl_device_registers.h"
#include "fsl_clock_manager.h"
#include "fsl_gpio_driver.h"
#include "fsl_smc_hal.h"

#ifdef __cplusplus
extern "C"{
#endif

void board_init(void);
void enable_clkout(int clktype);
bool dbg_init(uint32_t inst, uint32_t baud);

void usb_khci_irq_handler(void);

#ifdef __cplusplus
}
#endif

#endif // __TEENSY_LC_H
