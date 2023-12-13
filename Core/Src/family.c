/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 William D. Jones (thor0505@comcast.net),
 * Ha Thach (tinyusb.org)
 * Uwe Bonnes (bon@elektron.ikp.physik.tu-darmstadt.de)
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
 *
 * This file is part of the TinyUSB stack.
 */

#include "stm32l4xx_hal.h"
#include "board_api.h"
#include "board.h"

//--------------------------------------------------------------------+
// Forward USB interrupt events to TinyUSB IRQ Handler
//--------------------------------------------------------------------+

#if defined(USB_OTG_FS)
void OTG_FS_IRQHandler(void)
#else
void USB_IRQHandler(void)
#endif
{
  tud_int_handler(0);
}


//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM
//--------------------------------------------------------------------+

void board_init(void) {
  board_clock_init();

  // Enable All GPIOs clocks
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
#if defined(GPIOE)
  __HAL_RCC_GPIOE_CLK_ENABLE();
#endif
#if defined(GPIOF)
  __HAL_RCC_GPIOF_CLK_ENABLE();
#endif
#if defined(GPIOG)
  __HAL_RCC_GPIOG_CLK_ENABLE();
#endif
  __HAL_RCC_GPIOH_CLK_ENABLE();
  UART_CLK_EN();

  /* Enable USB power on Pwrctrl CR2 register */
  /* Enable Power Clock*/
  __HAL_RCC_PWR_CLK_ENABLE();

#if defined(PWR_CR5_R1MODE)
  /* Enable voltage range 1 boost mode for frequency above 80 Mhz */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
#endif

  /* Enable USB power on Pwrctrl CR2 register */
  HAL_PWREx_EnableVddUSB();

  GPIO_InitTypeDef  GPIO_InitStruct;


  // IOSV bit MUST be set to access GPIO port G[2:15] 
  __HAL_RCC_PWR_CLK_ENABLE();

#if defined(PWR_CR2_IOSV)
  HAL_PWREx_EnableVddIO2();
#endif

  /* Configure USB FS GPIOs */
  /* Configure DM DP Pins */
  GPIO_InitStruct.Pin = (GPIO_PIN_11 | GPIO_PIN_12);
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
#if defined(USB_OTG_FS)
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
#else
  GPIO_InitStruct.Alternate = GPIO_AF10_USB_FS;
#endif
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

#if defined(USB_OTG_FS)
  /* Configure VBUS Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Configure ID pin */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
#endif

  /* Enable USB FS Clocks */
#if defined(USB_OTG_FS)
  __HAL_RCC_USB_OTG_FS_CLK_ENABLE();
  board_vbus_sense_init();
#else
  __HAL_RCC_USB_CLK_ENABLE();
#endif

}

//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+


size_t board_get_unique_id(uint8_t id[], size_t max_len) {
  (void) max_len;
  volatile uint32_t * stm32_uuid = (volatile uint32_t *) UID_BASE;
  uint32_t* id32 = (uint32_t*) (uintptr_t) id;
  uint8_t const len = 12;

  id32[0] = stm32_uuid[0];
  id32[1] = stm32_uuid[1];
  id32[2] = stm32_uuid[2];

  return len;
}
