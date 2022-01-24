CROSS_COMPILE ?= riscv64-unknown-elf-
USE_PICOLIBC ?= 0

ifeq ($(shell $(CROSS_COMPILE)gcc --specs=picolibc.specs -E - 2>/dev/null >/dev/null </dev/null ; echo $$?),0)
USE_PICOLIBC = 1
endif
	
NUCLEI_SDK = lib/nuclei-sdk
GD32VF103_SDK_SOC = $(NUCLEI_SDK)/SoC/gd32vf103
GD32VF103_SDK_SOC_COMMON = $(NUCLEI_SDK)/SoC/gd32vf103/Common
GD32VF103_SDK_DRIVER = $(GD32VF103_SDK_SOC_COMMON)/Source/Drivers
SKIP_NANOLIB = 1

# List of git submodules that is included as part of the UF2 version
GIT_SUBMODULES = nuclei-sdk tinyusb

include ../make.mk

# Port Compiler Flags
CFLAGS += \
  -flto \
  -mcmodel=medany \
  -march=rv32imac \
  -mabi=ilp32 \
  -msave-restore \
  -mtune=size \
  -nostdlib \
  -nostartfiles \
  -DCFG_TUSB_MCU=OPT_MCU_GD32VF103 \
  -DDOWNLOAD_MODE=DOWNLOAD_MODE_FLASHXIP

ifeq ($(USE_PICOLIBC), 1)
CFLAGS += --specs=picolibc.specs
LD_SCRIPT_FLAG = -T $(TOP)/
endif

# suppress warning caused by vendor mcu driver
CFLAGS += -Wno-error=unused-parameter -Wno-stringop-overread

# Port source
PORT_SRC_C += \
	$(addprefix $(CURRENT_PATH)/, $(wildcard *.c)) \
	$(GD32VF103_SDK_DRIVER)/gd32vf103_rcu.c \
	$(GD32VF103_SDK_DRIVER)/gd32vf103_gpio.c \
	$(GD32VF103_SDK_DRIVER)/Usb/gd32vf103_usb_hw.c \
	$(GD32VF103_SDK_DRIVER)/gd32vf103_fmc.c \
	$(GD32VF103_SDK_DRIVER)/gd32vf103_usart.c 


ifeq ($(USE_PICOLIBC), 0)
PORT_SRC_C += \
	 $(GD32VF103_SDK_SOC_COMMON)/Source/Stubs/sbrk.c \
	 $(GD32VF103_SDK_SOC_COMMON)/Source/Stubs/close.c \
	 $(GD32VF103_SDK_SOC_COMMON)/Source/Stubs/isatty.c \
	 $(GD32VF103_SDK_SOC_COMMON)/Source/Stubs/fstat.c \
	 $(GD32VF103_SDK_SOC_COMMON)/Source/Stubs/lseek.c \
	 $(GD32VF103_SDK_SOC_COMMON)/Source/Stubs/read.c
endif

SRC_C += \
	$(PORT_SRC_C) \
	lib/tinyusb/src/portable/synopsys/dwc2/dcd_dwc2.c

SRC_S += \
  $(GD32VF103_SDK_SOC_COMMON)/Source/GCC/startup_gd32vf103.S \
  $(GD32VF103_SDK_SOC_COMMON)/Source/GCC/intexc_gd32vf103.S

# Port include
INC += \
	$(TOP)/$(NUCLEI_SDK)/NMSIS/Core/Include \
	$(TOP)/$(GD32VF103_SDK_SOC_COMMON)/Include \
	$(TOP)/$(GD32VF103_SDK_SOC_COMMON)/Include/Usb


include ../rules.mk

UF2_FAMILY_ID = 0x9af03e33
LD_FILES ?= $(PORT_DIR)/linker/gd32vf103xb.ld

# for flashing with flash-jlink
JLINK_IF = jtag

# flash target ROM bootloader
flash-dfu-util: $(BUILD)/$(OUTNAME).bin
	dfu-util -R -a 0 --dfuse-address 0x08000000 -D $<

#-------------- Self-update  --------------
self-update:
	@echo "not implemented yet"
