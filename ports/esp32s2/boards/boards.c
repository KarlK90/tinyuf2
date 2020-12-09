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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include "esp_rom_gpio.h"
#include "hal/gpio_ll.h"
#include "hal/usb_hal.h"
#include "soc/usb_periph.h"

#include "driver/periph_ctrl.h"
#include "driver/rmt.h"

#include "esp_partition.h"
#include "esp_ota_ops.h"
#include "board_api.h"

#ifndef TINYUF2_SELF_UPDATE
#include "tusb.h"
#endif

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+

static esp_timer_handle_t timer_hdl;

#ifdef NEOPIXEL_PIN
#include "led_strip.h"
static led_strip_t *strip;
#endif

#ifdef DOTSTAR_PIN_SCK
#include "led_strip_spi_apa102.h"
#endif

#ifdef LED_PIN
#include "driver/ledc.h"
ledc_channel_config_t ledc_channel =
{
  .channel    = LEDC_CHANNEL_0,
  .duty       = 0,
  .gpio_num   = LED_PIN,
  .speed_mode = LEDC_LOW_SPEED_MODE,
  .hpoint     = 0,
  .timer_sel  = LEDC_TIMER_0
};
#endif


extern int main(void);
static void configure_pins(usb_hal_context_t *usb);
static void internal_timer_cb(void* arg);

//--------------------------------------------------------------------+
// TinyUSB thread
//--------------------------------------------------------------------+

#ifdef TINYUF2_SELF_UPDATE

void app_main(void)
{
  main();
}

#else

// static task for usbd
// Increase stack size when debug log is enabled
#define USBD_STACK_SIZE     (4*1024)

static StackType_t  usb_device_stack[USBD_STACK_SIZE];
static StaticTask_t usb_device_taskdef;

// USB Device Driver task
// This top level thread process all usb events and invoke callbacks
void usb_device_task(void* param)
{
  (void) param;

  // RTOS forever loop
  while (1)
  {
    tud_task();
  }
}

void app_main(void)
{
  main();

  // Create a task for tinyusb device stack
  (void) xTaskCreateStatic( usb_device_task, "usbd", USBD_STACK_SIZE, NULL, configMAX_PRIORITIES-2, usb_device_stack, &usb_device_taskdef);
}

#endif

//--------------------------------------------------------------------+
// Board API
//--------------------------------------------------------------------+

void board_init(void)
{
#ifdef LED_PIN
  ledc_timer_config_t ledc_timer =
  {
    .duty_resolution = LEDC_TIMER_8_BIT      , // resolution of PWM duty
    .freq_hz         = 5000                  , // frequency of PWM signal
    .speed_mode      = LEDC_LOW_SPEED_MODE   , // timer mode
    .timer_num       = ledc_channel.timer_sel, // timer index
    .clk_cfg         = LEDC_AUTO_CLK         , // Auto select the source clock
  };
  ledc_timer_config(&ledc_timer);
  ledc_channel_config(&ledc_channel);
#endif

#ifdef NEOPIXEL_PIN
  #ifdef NEOPIXEL_POWER_PIN
  gpio_reset_pin(NEOPIXEL_POWER_PIN);
  gpio_set_direction(NEOPIXEL_POWER_PIN, GPIO_MODE_OUTPUT);
  gpio_set_level(NEOPIXEL_POWER_PIN, NEOPIXEL_POWER_STATE);
  #endif

  // WS2812 Neopixel driver with RMT peripheral
  rmt_config_t config = RMT_DEFAULT_CONFIG_TX(NEOPIXEL_PIN, RMT_CHANNEL_0);
  config.clk_div = 2; // set counter clock to 40MHz

  rmt_config(&config);
  rmt_driver_install(config.channel, 0, 0);

  led_strip_config_t strip_config = LED_STRIP_DEFAULT_CONFIG(NEOPIXEL_NUMBER, (led_strip_dev_t) config.channel);
  strip = led_strip_new_rmt_ws2812(&strip_config);
  strip->clear(strip, 100); // off led
  strip->set_brightness(strip, NEOPIXEL_BRIGHTNESS);
#endif

#ifdef DOTSTAR_PIN_SCK
  // Setup the IO for the APA DATA and CLK
  gpio_pad_select_gpio(DOTSTAR_PIN_DATA);
  gpio_pad_select_gpio(DOTSTAR_PIN_SCK);
  gpio_ll_input_disable(&GPIO, DOTSTAR_PIN_DATA);
  gpio_ll_input_disable(&GPIO, DOTSTAR_PIN_SCK);
  gpio_ll_output_enable(&GPIO, DOTSTAR_PIN_DATA);
  gpio_ll_output_enable(&GPIO, DOTSTAR_PIN_SCK);

  // Initialise SPI
  setupSPI(DOTSTAR_PIN_DATA, DOTSTAR_PIN_SCK);

  // Initialise the APA
  initAPA(DOTSTAR_BRIGHTNESS);
#endif

  // Set up timer
  const esp_timer_create_args_t periodic_timer_args = { .callback = internal_timer_cb };
  esp_timer_create(&periodic_timer_args, &timer_hdl);
}

void board_dfu_init(void)
{
  // USB Controller Hal init
  periph_module_reset(PERIPH_USB_MODULE);
  periph_module_enable(PERIPH_USB_MODULE);

  usb_hal_context_t hal = {
    .use_external_phy = false // use built-in PHY
  };
  usb_hal_init(&hal);
  configure_pins(&hal);
}

void board_dfu_complete(void)
{
  // Set partition OTA0 as bootable and reset
  esp_ota_set_boot_partition(esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL));
  esp_restart();
}

bool board_app_valid(void)
{
  // esp32s2 is always enter DFU mode
  return false;
}

void board_app_jump(void)
{
  // nothing to do
}

uint8_t board_usb_get_serial(uint8_t serial_id[16])
{
  // use factory default MAC as serial ID
  esp_efuse_mac_get_default(serial_id);
  return 6;
}

void board_led_write(uint32_t value)
{
  (void) value;

#ifdef LED_PIN
  ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, value);
  ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
#endif
}

void board_rgb_write(uint8_t const rgb[])
{
#ifdef NEOPIXEL_PIN
  for(uint32_t i=0; i<NEOPIXEL_NUMBER; i++)
  {
    strip->set_pixel(strip, i, rgb[0], rgb[1], rgb[2]);
  }
  strip->refresh(strip, 100);
#endif

#ifdef DOTSTAR_PIN_SCK
    setAPA(rgb[0], rgb[1], rgb[2]);
#endif
}

//--------------------------------------------------------------------+
// Timer
//--------------------------------------------------------------------+

static void internal_timer_cb(void*  arg)
{
  (void) arg;
  board_timer_handler();
}

void board_timer_start(uint32_t ms)
{
  esp_timer_stop(timer_hdl);
  esp_timer_start_periodic(timer_hdl, ms*1000);
}

void board_timer_stop(void)
{
  esp_timer_stop(timer_hdl);
}

//--------------------------------------------------------------------+
// Self Update
//--------------------------------------------------------------------+

//#ifdef TINYUF2_SELF_UPDATE
void board_self_update(const uint8_t * bootloader_bin, uint32_t bootloader_len)
{

}
//#endif

//--------------------------------------------------------------------+
// Helper
//--------------------------------------------------------------------+
static void configure_pins(usb_hal_context_t *usb)
{
  /* usb_periph_iopins currently configures USB_OTG as USB Device.
   * Introduce additional parameters in usb_hal_context_t when adding support
   * for USB Host.
   */
  for (const usb_iopin_dsc_t *iopin = usb_periph_iopins; iopin->pin != -1; ++iopin) {
    if ((usb->use_external_phy) || (iopin->ext_phy_only == 0)) {
      esp_rom_gpio_pad_select_gpio(iopin->pin);
      if (iopin->is_output) {
        esp_rom_gpio_connect_out_signal(iopin->pin, iopin->func, false, false);
      } else {
        esp_rom_gpio_connect_in_signal(iopin->pin, iopin->func, false);
        if ((iopin->pin != GPIO_FUNC_IN_LOW) && (iopin->pin != GPIO_FUNC_IN_HIGH)) {
          gpio_ll_input_enable(&GPIO, iopin->pin);
        }
      }
      esp_rom_gpio_pad_unhold(iopin->pin);
    }
  }
  if (!usb->use_external_phy) {
    gpio_set_drive_capability(USBPHY_DM_NUM, GPIO_DRIVE_CAP_3);
    gpio_set_drive_capability(USBPHY_DP_NUM, GPIO_DRIVE_CAP_3);
  }
}
