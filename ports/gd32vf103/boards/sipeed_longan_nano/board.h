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

#ifndef BOARD_H_
#define BOARD_H_

#define HXTAL_VALUE    ((uint32_t)8000000) /*!< value of the external oscillator in Hz */

//--------------------------------------------------------------------+
// LED
//--------------------------------------------------------------------+

#define LED_PORT              GPIOC
#define LED_PIN               GPIO_PIN_13
#define LED_STATE_ON          1

//--------------------------------------------------------------------+
// Neopixel
//--------------------------------------------------------------------+

//// Number of neopixels
#define NEOPIXEL_NUMBER       0

//#define NEOPIXEL_PORT         GPIOC
//#define NEOPIXEL_PIN          GPIO_PIN_0
//
//// Brightness percentage from 1 to 255
//#define NEOPIXEL_BRIGHTNESS   0x10


//--------------------------------------------------------------------+
// Flash
//--------------------------------------------------------------------+

// Flash size of the board
#define BOARD_FLASH_SECTORS 128
#define BOARD_FLASH_SIZE (BOARD_FLASH_SECTORS * 1024)

//--------------------------------------------------------------------+
// USB UF2
//--------------------------------------------------------------------+

#define USB_VID           0x239A
#define USB_PID           0x0069
#define USB_MANUFACTURER  "GD"
#define USB_PRODUCT       "LonganNano"

#define UF2_PRODUCT_NAME  USB_MANUFACTURER " " USB_PRODUCT
#define UF2_BOARD_ID      "Longan-Nano"
#define UF2_VOLUME_LABEL  "Longan Nano"
#define UF2_INDEX_URL     "https://longan.sipeed.com/en/"

#define USB_NO_VBUS_PIN   1

//--------------------------------------------------------------------+
// UART
//--------------------------------------------------------------------+

#define UART_DEV              USART0
#define UART_GPIO_PORT        GPIOA
#define UART_TX_PIN           GPIO_PIN_9
#define UART_RX_PIN           GPIO_PIN_10

/* sipeed longan nano board UART com port */
#define GD32_COM0                        USART0
#define GD32_COM_CLK                     RCU_USART0
#define GD32_COM_TX_PIN                  GPIO_PIN_9
#define GD32_COM_RX_PIN                  GPIO_PIN_10
#define GD32_COM_TX_GPIO_PORT            GPIOA
#define GD32_COM_RX_GPIO_PORT            GPIOA
#define GD32_COM_TX_GPIO_CLK             RCU_GPIOA
#define GD32_COM_RX_GPIO_CLK             RCU_GPIOA

//--------------------------------------------------------------------+
// RCC Clock
//--------------------------------------------------------------------+
static inline void clock_init(void)
{
}

#endif
