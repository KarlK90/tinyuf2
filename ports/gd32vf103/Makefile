UF2_FAMILY_ID = 0x57755a57
CROSS_COMPILE = riscv32-unknown-elf-

NUCLEI_SDK = lib/nuclei-sdk
GD32VF103_SDK_SOC_COMMON = $(NUCLEI_SDK)/SoC/gd32vf103/Common
GD32VF103_SDK_DRIVER = $(GD32VF103_SDK_SOC_COMMON)/Source/Drivers
SKIP_NANOLIB = 1

# List of git submodules that is included as part of the UF2 version
GIT_SUBMODULES = nuclei-sdk tinyusb

include ../make.mk

DEBUG = 1
LOG = 0

# Port Compiler Flags
CFLAGS += \
  -gdwarf-4 \
  -march=rv32imac \
  -mabi=ilp32 \
  -mcmodel=medlow \
  -mstrict-align \
  -nostdlib -nostartfiles \
  -DCFG_TUSB_MCU=OPT_MCU_GD32VF103 \
  -DDOWNLOAD_MODE=DOWNLOAD_MODE_FLASHXIP \
  #-D__riscv_flen=0 

# suppress warning caused by vendor mcu driver
CFLAGS += -Wno-error=cast-align -Wno-error=unused-parameter

LD_FILES ?= $(PORT_DIR)/linker/gd32vf103xb.ld

# Port source
PORT_SRC_C += \
	$(addprefix $(CURRENT_PATH)/, $(wildcard *.c)) \
	$(GD32VF103_SDK_DRIVER)/gd32vf103_rcu.c \
	$(GD32VF103_SDK_DRIVER)/gd32vf103_gpio.c \
	$(GD32VF103_SDK_DRIVER)/Usb/gd32vf103_usb_hw.c \
	$(GD32VF103_SDK_DRIVER)/gd32vf103_fmc.c \
	$(GD32VF103_SDK_DRIVER)/gd32vf103_usart.c \
	$(GD32VF103_SDK_SOC_COMMON)/Source/Stubs/sbrk.c \
	$(GD32VF103_SDK_SOC_COMMON)/Source/Stubs/close.c \
	$(GD32VF103_SDK_SOC_COMMON)/Source/Stubs/isatty.c \
	$(GD32VF103_SDK_SOC_COMMON)/Source/Stubs/fstat.c \
	$(GD32VF103_SDK_SOC_COMMON)/Source/Stubs/lseek.c \
	$(GD32VF103_SDK_SOC_COMMON)/Source/Stubs/read.c \

SRC_C += \
	$(PORT_SRC_C) \
	lib/tinyusb/src/portable/st/synopsys/dcd_synopsys.c

SRC_S += \
  $(GD32VF103_SDK_SOC_COMMON)/Source/GCC/startup_gd32vf103.S \
  $(GD32VF103_SDK_SOC_COMMON)/Source/GCC/intexc_gd32vf103.S

# Port include
INC += \
	$(TOP)/$(NUCLEI_SDK)/NMSIS/Core/Include \
	$(TOP)/$(GD32VF103_SDK_SOC_COMMON)/Include \
	$(TOP)/$(GD32VF103_SDK_SOC_COMMON)/Include/Usb

include ../rules.mk

# flash target ROM bootloader
flash-dfu-util: $(BUILD)/$(OUTNAME).bin
	dfu-util -R -a 0 --dfuse-address 0x08000000 -D $<

#-------------- Self-update  --------------
self-update:
	@echo "not implemented yet"
