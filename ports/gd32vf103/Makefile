UF2_FAMILY_ID = 0x57755a57
CROSS_COMPILE = riscv32-unknown-elf-

ST_HAL_DRIVER = lib/st/stm32f4xx_hal_driver
ST_CMSIS = lib/st/cmsis_device_f4
NUCLEI_SDK = lib/nuclei-sdk
GD32VF103_SDK_SOC_COMMON = $(NUCLEI_SDK)/SoC/gd32vf103/Common
CMSIS_5 = lib/CMSIS_5
SKIP_NANOLIB = 1

# List of git submodules that is included as part of the UF2 version
GIT_SUBMODULES = st/cmsis_device_f4 st/stm32f4xx_hal_driver tinyusb

include ../make.mk

# Port Compiler Flags
CFLAGS += \
  -flto \
  -march=rv32imac \
  -mabi=ilp32 \
  -mcmodel=medlow \
  -mstrict-align \
  -nostdlib -nostartfiles \
  -DCFG_TUSB_MCU=OPT_MCU_STM32F4 \
  -D__riscv_flen=0

# suppress warning caused by vendor mcu driver
CFLAGS += -Wno-error=cast-align -Wno-error=unused-parameter

LD_FILES ?= $(PORT_DIR)/linker/gd32vf103xb.ld

# Port source
PORT_SRC_C += \
	$(addprefix $(CURRENT_PATH)/, $(wildcard *.c)) \
	$(ST_CMSIS)/Source/Templates/system_stm32f4xx.c \
	$(ST_HAL_DRIVER)/Src/stm32f4xx_hal.c \
	$(ST_HAL_DRIVER)/Src/stm32f4xx_hal_cortex.c \
	$(ST_HAL_DRIVER)/Src/stm32f4xx_hal_rcc.c \
	$(ST_HAL_DRIVER)/Src/stm32f4xx_hal_gpio.c \
	$(ST_HAL_DRIVER)/Src/stm32f4xx_hal_flash.c \
	$(ST_HAL_DRIVER)/Src/stm32f4xx_hal_flash_ex.c \
	$(ST_HAL_DRIVER)/Src/stm32f4xx_hal_uart.c

SRC_C += \
	$(PORT_SRC_C) \
	lib/tinyusb/src/portable/st/synopsys/dcd_synopsys.c \

SRC_S += \
  $(GD32VF103_SDK_SOC_COMMON)/Source/GCC/startup_gd32vf103.S \
  $(GD32VF103_SDK_SOC_COMMON)/Source/GCC/intexc_gd32vf103.S

# Port include
INC += \
	$(TOP)/$(NUCLEI_SDK)/NMSIS/Core/Include \
	$(TOP)/$(GD32VF103_SDK_SOC_COMMON)/Include

#	$(TOP)/$(ST_CMSIS)/Include \
#	$(TOP)/$(ST_HAL_DRIVER)/Inc \
	

#$(TOP)/$(CMSIS_5)/CMSIS/Core/Include

include ../rules.mk

# flash target ROM bootloader
flash-dfu-util: $(BUILD)/$(OUTNAME).bin
	dfu-util -R -a 0 --dfuse-address 0x08000000 -D $<

#-------------- Self-update  --------------
self-update:
	@echo "not implemented yet"
