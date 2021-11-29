/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 Ha Thach for Adafruit Industries
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "board_api.h"
#include "tusb.h"  // for logging

#define FLASH_CACHE_INVALID_ADDR 0xffffffff

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+

#if !defined(BOARD_FLASH_SECTORS)
#define BOARD_FLASH_SECTORS 128
#endif

#define BOARD_FLASH_SECTOR_SIZE 1024

static bool is_blank(uint32_t addr, uint32_t size) {
  for (uint32_t i = 0; i < size; i += sizeof(uint32_t)) {
    if (*(uint32_t *)(addr + i) != 0xffffffff) {
      return false;
    }
  }
  return true;
}

// Bitfield
static uint16_t erased_sectors[BOARD_FLASH_SECTORS / 16];

void flash_write(uint32_t dst, const uint8_t *src, int len) {
  uint32_t addr = BOARD_FLASH_APP_START;
  uint32_t sector = 0;
  bool erased = false;

  for (uint32_t i = 0; i < BOARD_FLASH_SECTORS; i++) {
    if (addr + BOARD_FLASH_SECTOR_SIZE > dst) {
      sector = i + 1;
      uint16_t sector_mask = (uint16_t)(1 << (i % 16));
      uint32_t sector_index = i / 16;
      erased = (sector_mask & erased_sectors[sector_index]) != 0;
      erased_sectors[sector_index] |= sector_mask;
      break;
    }
    addr += BOARD_FLASH_SECTOR_SIZE;
  }

  if (sector == 0) {
    TU_LOG1("invalid sector");
  }

  fmc_unlock();
  ob_unlock();
  fmc_flag_clear(FMC_FLAG_END);
  fmc_flag_clear(FMC_FLAG_WPERR);
  fmc_flag_clear(FMC_FLAG_PGERR);

  if (!erased && !is_blank(addr, BOARD_FLASH_SECTOR_SIZE)) {
    TU_LOG1("Erase: %08lX size = %u\n", addr, BOARD_FLASH_SECTOR_SIZE);

    for (uint32_t i = 0; i < 1; i++) {
      if (fmc_page_erase(addr + (i * BOARD_FLASH_SECTOR_SIZE)) != FMC_READY) {
        TU_LOG1("Waiting on last operation failed\n");
        return;
      };
    }

    if (!is_blank(addr, BOARD_FLASH_SECTOR_SIZE)) {
      TU_LOG1("failed to erase!\n");
    }
  }

  for (int i = 0; i < len; i += sizeof(uint32_t)) {
    uint32_t data = *((uint32_t *)((void *)(src + i)));
    if (fmc_word_program(dst + i, data) != FMC_READY) {
      TU_LOG1("Failed to write flash at address %08lX\n", dst + i);
      break;
    };
  }

  if (memcmp((void *)dst, src, len) != 0) {
    TU_LOG1("failed to write\n");
  }
}

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+
void board_flash_init(void) {}

uint32_t board_flash_size(void) { return BOARD_FLASH_SIZE; }

void board_flash_read(uint32_t addr, void *buffer, uint32_t len) {
  TU_LOG1("Board Flash Read\n");
  memcpy(buffer, (void *)addr, len);
}

void board_flash_flush(void) {}

void board_flash_write(uint32_t addr, void const *data, uint32_t len) {
  TU_LOG1("Board Flash Write\n");
  flash_write(addr, data, len);
}

void board_flash_erase_app(void) {
  // TODO implement later
}

#ifdef TINYUF2_SELF_UPDATE
void board_self_update(const uint8_t *bootloader_bin, uint32_t bootloader_len) {
  (void)bootloader_bin;
  (void)bootloader_len;
}
#endif
