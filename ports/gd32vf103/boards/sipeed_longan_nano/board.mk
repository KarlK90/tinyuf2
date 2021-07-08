CFLAGS += \
  -DSTM32F411xE \
  -DHSE_VALUE=25000000U

# For flash-jlink target
JLINK_DEVICE = stm32f411ce

flash: flash-jlink
erase: erase-jlink
