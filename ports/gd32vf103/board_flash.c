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

#ifndef BOARD_FLASH_SECTORS
#define BOARD_FLASH_SECTORS 128
#endif

#define BOARD_FLASH_SECTOR_SIZE 1024

//#define BOARD_FIRST_FLASH_SECTOR_TO_ERASE 0

#define APP_LOAD_ADDRESS 0x08010000

static uint8_t erasedSectors[BOARD_FLASH_SECTORS];

uint32_t flash_func_sector_size(unsigned sector) {
  if (sector < BOARD_FLASH_SECTORS) {
    return BOARD_FLASH_SECTOR_SIZE;
  }

  return 0;
}

static bool is_blank(uint32_t addr, uint32_t size) {
  for (uint32_t i = 0; i < size; i += sizeof(uint32_t)) {
    if (*(uint32_t *)(addr + i) != 0xffffffff) {
      return false;
    }
  }
  return true;
}

void flash_write(uint32_t dst, const uint8_t *src, int len) {
  // // assume sector 0 (bootloader) is same size as sector 1
  // uint32_t addr = flash_func_sector_size(0) + (APP_LOAD_ADDRESS & 0xfff00000);
  // uint32_t sector = 0;
  // int erased = false;
  // uint32_t size = 0;

  // for (unsigned i = 0; i < BOARD_FLASH_SECTORS; i++) {
  //   size = flash_func_sector_size(i);
  //   if (addr + size > dst) {
  //     sector = i + 1;
  //     erased = erasedSectors[i];
  //     erasedSectors[i] =
  //         1;  // don't erase anymore - we will continue writing here!
  //     break;
  //   }
  //   addr += size;
  // }

  // if (sector == 0) {
  //   TU_LOG1("invalid sector");
  // }

  // fmc_unlock();

  // if (!erased && !is_blank(addr, size)) {
  //   TU_LOG1("Erase: %08lX size = %lu\n", addr, size);

  //   if (fmc_page_erase(sector) != FMC_READY) {
  //     TU_LOG1("Waiting on last operation failed");
  //     return;
  //   };

  //   if (!is_blank(addr, size)) {
  //     TU_LOG1("failed to erase!");
  //   }
  // }

  // for (int i = 0; i < len; i += 4) {
  //  /* if (fmc_word_program(dst + i, (uint64_t)(*(uint32_t *)(src + i))) !=
  //       FMC_READY) { TU_LOG1("Failed to write flash at
  //      address %08lX", dst + i); break;
  //   };
  //   if (FLASH_WaitForLastOperation(HAL_MAX_DELAY) != HAL_OK) {
  //     TU_LOG1("Waiting on last operation failed");
  //     return;
  //   };*/
  // }

  // if (memcmp((void *)dst, src, len) != 0) {
  //   TU_LOG1("failed to write");
  // }
}

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+
void board_flash_init(void) {}

uint32_t board_flash_size(void) { return BOARD_FLASH_SIZE; }

void board_flash_read(uint32_t addr, void *buffer, uint32_t len) {
  memcpy(buffer, (void *)addr, len);
}

void board_flash_flush(void) {}

// TODO not working quite yet
void board_flash_write(uint32_t addr, void const *data, uint32_t len) {
  // TODO skip matching contents
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
