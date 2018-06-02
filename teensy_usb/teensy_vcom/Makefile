######################################
# target
######################################
TARGET = virtual_com
CPU = CPU_MKL26Z64VFT4
DBGFLAG = 0

#######################################
# pathes
#######################################
BUILD_DIR = ./build
SDK_DIR = /home/brian/Projects/Kinetis/SDK_1.3_MKL26Z64xxx4
TEENSY_DIR = /home/brian/Projects/Kinetis/projects/teensy_common

######################################
# source
######################################
C_SOURCES =\
	virtual_com.c\
	usb_descriptor.c\
	$(TEENSY_DIR)/teensy_lc.c\
	$(TEENSY_DIR)/tlc_khci_dev.c\
	$(SDK_DIR)/platform/devices/MKL26Z4/startup/system_MKL26Z4.c\
	$(SDK_DIR)/platform/devices/startup.c\
	$(SDK_DIR)/platform/utilities/src/print_scan.c\
	$(SDK_DIR)/platform/utilities/src/fsl_debug_console.c\
	$(SDK_DIR)/platform/utilities/src/fsl_misc_utilities.c

ASM_SOURCES =\
	$(SDK_DIR)/platform/devices/MKL26Z4/startup/gcc/startup_MKL26Z4.S

LIBS = \
	$(SDK_DIR)/lib/ksdk_platform_lib/armgcc/KL26Z4/release/libksdk_platform.a\
	$(SDK_DIR)/usb/usb_core/device/lib/bm/armgcc/MKL26Z4/release/libusbd_bm.a

#######################################
# binaries
#######################################
CC = arm-none-eabi-gcc
AS = arm-none-eabi-gcc -x assembler-with-cpp
CP = arm-none-eabi-objcopy
AR = arm-none-eabi-ar
SZ = arm-none-eabi-size
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

#######################################
# FLAGS
#######################################
INCLUDES = -I.
INCLUDES += -I$(TEENSY_DIR)
INCLUDES += -I$(SDK_DIR)/platform/osa/inc
INCLUDES += -I$(SDK_DIR)/platform/utilities/inc
INCLUDES += -I$(SDK_DIR)/platform/CMSIS/Include
INCLUDES += -I$(SDK_DIR)/platform/devices
INCLUDES += -I$(SDK_DIR)/platform/devices/MKL26Z4/include
INCLUDES += -I$(SDK_DIR)/platform/devices/MKL26Z4/startup
INCLUDES += -I$(SDK_DIR)/platform/hal/inc
INCLUDES += -I$(SDK_DIR)/platform/drivers/inc
INCLUDES += -I$(SDK_DIR)/platform/drivers/src/gpio
INCLUDES += -I$(SDK_DIR)/platform/system/inc
INCLUDES += -I$(SDK_DIR)/usb/usb_core/include
INCLUDES += -I$(SDK_DIR)/usb/usb_core/device/include
INCLUDES += -I$(SDK_DIR)/usb/usb_core/device/include/MKL26Z4
INCLUDES += -I$(SDK_DIR)/usb/adapter/sources
INCLUDES += -I$(SDK_DIR)/usb/adapter/sources/sdk
INCLUDES += -I$(SDK_DIR)/usb/usb_core/device/sources/classes/include
INCLUDES += -I$(SDK_DIR)/usb/usb_core/device/sources/controller/khci
INCLUDES += -I$(SDK_DIR)/usb/usb_core/hal
#INCLUDES += -I$(SDK_DIR)/usb/usb_core/device/sources/classes/common
#INCLUDES += -I$(SDK_DIR)/usb/usb_core/device/sources/classes/include/config
#INCLUDES += -I$(SDK_DIR)/usb/usb_core/device/sources/classes/cdc
#INCLUDES += -I$(SDK_DIR)/usb/usb_core/device/sources/controller

ASM_DEFINES =
ASM_FLAGS = -mcpu=cortex-m0plus -mthumb -Wall -fno-common -ffunction-sections\
			-fdata-sections -ffreestanding -fno-builtin -mapcs -std=gnu99\
			$(INCLUDES) $(ASM_DEFINES)

C_DEFINES = -D_DEBUG=$(DBGFLAG) -D$(CPU)
C_FLAGS = -O3 -Os -mcpu=cortex-m0plus -mthumb -MMD -MP -Wall -fno-common\
		  -ffunction-sections -fdata-sections -ffreestanding -fno-builtin\
		  -mapcs -std=gnu99\
		  $(INCLUDES) $(C_DEFINES)

LDSCRIPT = $(SDK_DIR)/platform/devices/MKL26Z4/linker/gcc/MKL26Z64xxx4_flash.ld
LD_FLAGS = $(C_FLAGS)\
		  -T$(LDSCRIPT)\
		  -static --specs=nano.specs -Wall\
		  -Xlinker --gc-sections\
		  -Xlinker -static\
		  -Xlinker -z\
		  -Xlinker muldefs\
		  -Xlinker --defsym=__heap_size__=0x200\
		  -Xlinker --defsym=__stack_size__=0x100\
		  -Xlinker -Map=$(BUILD_DIR)/$(TARGET).map\
		  -Wl,--start-group $(LIBS) -lm -lc -lgcc -lnosys -Wl,--end-group

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

#######################################
# build the application
#######################################
# list of C program objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.S=.o)))
vpath %.S $(sort $(dir $(ASM_SOURCES)))


$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	$(CC) -c $(C_FLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.S Makefile | $(BUILD_DIR)
	$(AS) -c $(ASM_FLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LD_FLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@

$(BUILD_DIR):
	mkdir -p $@

#######################################
# clean up
#######################################
clean:
	rm -fR $(BUILD_DIR)

#######################################
# flash
#######################################
flash:
	teensy_loader_cli --mcu=mkl26z64 $(BUILD_DIR)/$(TARGET).hex

# *** EOF ***
