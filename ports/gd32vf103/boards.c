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
#include "tusb.h"

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+

// According to GD32VF103 user manual clock tree:
// Systick clock = AHB clock / 4.
#define SYSTICK_CLOCK (SystemCoreClock / 4)
#define TIMER_TICKS (SYSTICK_CLOCK / 1000)

#define GD32_UUID ((uint32_t *)0x1FFFF7E8)

void board_init(void) {
  /* Disable interrupts during init */
  __disable_irq();
  rcu_periph_clock_enable(RCU_GPIOA);
  rcu_periph_clock_enable(RCU_GPIOB);
  rcu_periph_clock_enable(RCU_GPIOC);
  rcu_periph_clock_enable(RCU_GPIOD);
  rcu_periph_clock_enable(RCU_AF);

#ifdef BUTTON_PIN
  gpio_init(BUTTON_PORT, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, BUTTON_PIN);
#endif

#ifdef LED_PIN
  gpio_init(LED_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, LED_PIN);
  board_led_write(0);
#endif

#if NEOPIXEL_NUMBER
  __enable_mcycle_counter();
  gpio_init(NEOPIXEL_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, NEOPIXEL_PIN);
#endif

#if defined(UART_DEV) && CFG_TUSB_DEBUG
  /* enable GPIO TX and RX clock */
  rcu_periph_clock_enable(GD32_COM_TX_GPIO_CLK);
  rcu_periph_clock_enable(GD32_COM_RX_GPIO_CLK);

  /* enable USART clock */
  rcu_periph_clock_enable(GD32_COM_CLK);

  /* connect port to USARTx_Tx */
  gpio_init(GD32_COM_TX_GPIO_PORT, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ,
            GD32_COM_TX_PIN);

  /* connect port to USARTx_Rx */
  gpio_init(GD32_COM_RX_GPIO_PORT, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ,
            GD32_COM_RX_PIN);

  /* USART configure */
  usart_deinit(UART_DEV);
  usart_baudrate_set(UART_DEV, 115200U);
  usart_word_length_set(UART_DEV, USART_WL_8BIT);
  usart_stop_bit_set(UART_DEV, USART_STB_1BIT);
  usart_parity_config(UART_DEV, USART_PM_NONE);
  usart_hardware_flow_rts_config(UART_DEV, USART_RTS_DISABLE);
  usart_hardware_flow_cts_config(UART_DEV, USART_CTS_DISABLE);
  usart_receive_config(UART_DEV, USART_RECEIVE_ENABLE);
  usart_transmit_config(UART_DEV, USART_TRANSMIT_ENABLE);
  usart_enable(UART_DEV);

#endif
  /* Enable interrupts globaly */
  __enable_irq();
}

void board_dfu_init(void) {
  /* USB D+ and D- pins don't need to be configured. */
  /* Configure VBUS Pin */
#if !defined(USB_NO_VBUS_PIN)
  gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_9);
#endif

  /* This for ID line debug */
  // gpio_init(GPIOA, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_10);

  /* Enable USB OTG clock */
  usb_rcu_config();

  /* Reset USB OTG peripheral */
  rcu_periph_reset_enable(RCU_USBFSRST);
  rcu_periph_reset_disable(RCU_USBFSRST);

  /* Configure USBFS IRQ */
  ECLIC_Register_IRQ(USBFS_IRQn, ECLIC_NON_VECTOR_INTERRUPT,
                     ECLIC_POSTIVE_EDGE_TRIGGER, 3, 0, NULL);

  /* Retrieve otg core registers */
  usb_gr *otg_core_regs = (usb_gr *)(USBFS_REG_BASE + USB_REG_OFFSET_CORE);

#ifdef USB_NO_VBUS_PIN
  /* Disable VBUS sense*/
  otg_core_regs->GCCFG |= GCCFG_VBUSIG | GCCFG_PWRON | GCCFG_VBUSBCEN;
#else
  /* Enable VBUS sense via pin PA9 */
  otg_core_regs->GCCFG |= GCCFG_VBUSIG | GCCFG_PWRON | GCCFG_VBUSBCEN;
  otg_core_regs->GCCFG &= ~GCCFG_VBUSIG;
#endif
}

#include "gd32vf103_dbg.h"

#define DBG_KEY_UNLOCK 0x4B5A6978
#define DBG_CMD_RESET 0x1

#define DBG_KEY REG32(DBG + 0x0C)
#define DBG_CMD REG32(DBG + 0x08)

void gd32vf103_reset(void) {
  /* The MTIMER unit of the GD32VF103 doesn't have the MSFRST register to
   * generate a software reset request. BUT instead two undocumented registers
   * in the debug peripheral that allow issueing a software reset. */
  DBG_KEY = DBG_KEY_UNLOCK;
  DBG_CMD = DBG_CMD_RESET;
}

void board_reset(void) { gd32vf103_reset(); }

void board_dfu_complete(void) { gd32vf103_reset(); }

bool board_app_valid(void) {
  const uint32_t op_code = *(volatile uint32_t const *)BOARD_FLASH_APP_START;

  // op-code for CSRCI mstatus, 8 which disables all interrupts
  return op_code == 0x30047073;
}

void board_app_jump(void) {
  __disable_irq();

  volatile uint32_t const *app_vector =
      (volatile uint32_t const *)BOARD_FLASH_APP_START;

#ifdef BUTTON_PIN
  gpio_deinit(BUTTON_PORT);
#endif

#ifdef LED_PIN
  gpio_deinit(LED_PORT);
#endif

#if NEOPIXEL_NUMBER
  gpio_deinit(NEOPIXEL_PORT);
#endif

#if defined(UART_DEV) && CFG_TUSB_DEBUG
  usart_deinit(UART_DEV);
  gpio_deinit(UART_GPIO_PORT);
#endif

  // Jump to Application Entry
  asm("jr %0" ::"r"(app_vector));
}

uint8_t board_usb_get_serial(uint8_t serial_id[16]) {
  uint8_t const len = 12;
  memcpy(serial_id, GD32_UUID, len);
  return len;
}

//--------------------------------------------------------------------+
// LED pattern
//--------------------------------------------------------------------+

void board_led_write(uint32_t state) {
#if defined(LED_PIN)
  gpio_bit_write(LED_PORT, LED_PIN, state ? LED_STATE_ON : (1 - LED_STATE_ON));
#else
  (void)state;
#endif
}

#if NEOPIXEL_NUMBER

static inline uint8_t apply_percentage(uint8_t brightness) {
  return (uint8_t)((brightness * NEOPIXEL_BRIGHTNESS) >> 8);
}

#define MAGIC_800_INT 900000   // ~1.11 us -> 1.2  field
#define MAGIC_800_T0H 2800000  // ~0.36 us -> 0.44 field
#define MAGIC_800_T1H 1350000  // ~0.74 us -> 0.84 field

void board_rgb_write(uint8_t const rgb[]) {
  // assumes 800_000Hz frequency
  // Theoretical values here are 800_000 -> 1.25us, 2500000->0.4us,
  // 1250000->0.8us
  uint32_t const interval = SystemCoreClock / MAGIC_800_INT;
  uint32_t const t0 = SystemCoreClock / MAGIC_800_T0H;
  uint32_t const t1 = SystemCoreClock / MAGIC_800_T1H;

  // neopixel color order is GRB
  uint8_t const colors[3] = {apply_percentage(rgb[1]), apply_percentage(rgb[0]),
                             apply_percentage(rgb[2])};

  __disable_irq();
  uint32_t start;
  uint32_t cyc;

  __RV_CSR_WRITE(CSR_MCYCLEH, 0);
  __RV_CSR_WRITE(CSR_MCYCLE, 0);

  for (uint32_t i = 0; i < NEOPIXEL_NUMBER; i++) {
    uint8_t const *color_pointer = colors;
    uint8_t const *const color_pointer_end = color_pointer + 3;
    uint8_t color = *color_pointer++;
    uint8_t color_mask = 0x80;

    while (true) {
      cyc = (color & color_mask) ? t1 : t0;
      start = __RV_CSR_READ(CSR_MCYCLE);

      gpio_bit_set(NEOPIXEL_PORT, NEOPIXEL_PIN);
      while ((__RV_CSR_READ(CSR_MCYCLE) - start) < cyc)
        ;

      gpio_bit_reset(NEOPIXEL_PORT, NEOPIXEL_PIN);
      while ((__RV_CSR_READ(CSR_MCYCLE) - start) < interval)
        ;

      if (!(color_mask >>= 1)) {
        if (color_pointer >= color_pointer_end) {
          break;
        }
        color = *color_pointer++;
        color_mask = 0x80;
      }
    }
  }

  __enable_irq();
}

#else

void board_rgb_write(uint8_t const rgb[]) { (void)rgb; }

#endif

//--------------------------------------------------------------------+
// Timer
//--------------------------------------------------------------------+

static uint64_t timer_ticks;

void board_timer_start(uint32_t ms) {
  timer_ticks = TIMER_TICKS * ms;
  SysTick_Config(timer_ticks);
}

void board_timer_stop(void) { SysTimer_Stop(); }

void eclic_mtip_handler(void) {
  SysTick_Reload(timer_ticks);
  board_timer_handler();
}

int board_uart_write(void const *buf, int len) {
#if defined(UART_DEV) && CFG_TUSB_DEBUG

  while (len--) {
    usart_write(UART_DEV, *(uint8_t *)buf);
    buf++;
  }
  return len;
#else
  (void)buf;
  (void)len;
  return 0;
#endif
}

#ifndef TINYUF2_SELF_UPDATE

// Forward USB interrupt events to TinyUSB IRQ Handler
void USBFS_IRQHandler(void) { tud_int_handler(0); }

#endif
