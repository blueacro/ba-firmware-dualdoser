/*
 * Copyright (c) 2022, StackFoundry LLC
 * Copyright (c) 2017, Alex Taradov <alex@taradov.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*- Includes ----------------------------------------------------------------*/
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdalign.h>
#include <string.h>
#include "samd11.h"
#include "hal_gpio.h"
#include "nvm_data.h"
#include "usb.h"
#include "usb_vendor.h"

/*- Definitions -------------------------------------------------------------*/
#define USB_BUFFER_SIZE 64

HAL_GPIO_PIN(RED, A, 17);
HAL_GPIO_PIN(GREEN, A, 22);
HAL_GPIO_PIN(BLUE, A, 23);

HAL_GPIO_PIN(PUMP1, A, 6);
HAL_GPIO_PIN(PUMP2, A, 7);

HAL_GPIO_PIN(FLOAT1, A, 8);
HAL_GPIO_PIN(FLOAT2, A, 9);

HAL_GPIO_PIN(BUTTON, A, 16);

HAL_GPIO_PIN(PWRIN, A, 2);

#define STATUS_INACTIVE_STATE 0 // 0 - Low, 1 - High, 2 - Hi-Z
#define STATUS_ACTIVE_STATE 1   // 0 - Low, 1 - High, 2 - Hi-Z
#define STATUS_RISING_EDGE 0    // ms
#define STATUS_FALLING_EDGE 0   // ms

/*- Variables ---------------------------------------------------------------*/
static alignas(4) uint8_t app_recv_buffer[USB_BUFFER_SIZE];
static alignas(4) uint8_t app_send_buffer[USB_BUFFER_SIZE];
static int app_recv_buffer_size = 0;
static int app_recv_buffer_ptr = 0;
static int app_send_buffer_ptr = 0;
static bool app_send_buffer_free = true;
static bool app_send_zlp = false;
static int app_system_time = 0;
static int app_uart_timeout = 0;
static bool app_status = false;
static int app_status_timeout = 0;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static void sys_init(void)
{
  uint32_t sn = 0;

  /*
  configure oscillator for crystal-free USB operation (USBCRM / USB Clock Recovery Mode)
  */
  uint32_t coarse, fine;

  NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CACHEDIS | NVMCTRL_CTRLB_RWS(2);

  SYSCTRL->INTFLAG.reg = SYSCTRL_INTFLAG_BOD33RDY | SYSCTRL_INTFLAG_BOD33DET |
                         SYSCTRL_INTFLAG_DFLLRDY;

  coarse = NVM_READ_CAL(NVM_DFLL48M_COARSE_CAL);
  fine = NVM_READ_CAL(NVM_DFLL48M_FINE_CAL);

  SYSCTRL->DFLLCTRL.reg = 0; // See Errata 9905
  while (0 == (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY))
    ;

  SYSCTRL->DFLLMUL.reg = SYSCTRL_DFLLMUL_MUL(48000);
  SYSCTRL->DFLLVAL.reg = SYSCTRL_DFLLVAL_COARSE(coarse) | SYSCTRL_DFLLVAL_FINE(fine);

  SYSCTRL->DFLLCTRL.reg = SYSCTRL_DFLLCTRL_ENABLE | SYSCTRL_DFLLCTRL_USBCRM |
                          SYSCTRL_DFLLCTRL_MODE | SYSCTRL_DFLLCTRL_CCDIS;

  while (0 == (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY))
    ;

  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(0) | GCLK_GENCTRL_SRC(GCLK_SOURCE_DFLL48M) |
                      GCLK_GENCTRL_RUNSTDBY | GCLK_GENCTRL_GENEN;
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY)
    ;

  sn ^= *(volatile uint32_t *)0x0080a00c;
  sn ^= *(volatile uint32_t *)0x0080a040;
  sn ^= *(volatile uint32_t *)0x0080a044;
  sn ^= *(volatile uint32_t *)0x0080a048;

  for (int i = 0; i < 8; i++)
    usb_serial_number[i] = "0123456789ABCDEF"[(sn >> (i * 4)) & 0xf];

  usb_serial_number[9] = 0;
}

//-----------------------------------------------------------------------------
static void sys_time_init(void)
{
  SysTick->VAL = 0;
  SysTick->LOAD = F_CPU / 1000ul;
  SysTick->CTRL = SysTick_CTRL_ENABLE_Msk;
  app_system_time = 0;
}

//-----------------------------------------------------------------------------
static void sys_time_task(void)
{
  if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
    app_system_time++;
}

//-----------------------------------------------------------------------------
static int get_system_time(void)
{
  return app_system_time;
}

//-----------------------------------------------------------------------------
static void update_status(bool status)
{

  app_status = status;
}

//-----------------------------------------------------------------------------
static void status_task(void)
{
  if (app_status_timeout && get_system_time() > app_status_timeout)
  {
    app_status_timeout = 0;
  }
  if (app_system_time % 100 == 0) {
    HAL_GPIO_BLUE_toggle();
  }

  int pwr = HAL_GPIO_PWRIN_read();
  if (!pwr) {
    HAL_GPIO_RED_set();
  } else {
    HAL_GPIO_RED_clr();
  }
}

//-----------------------------------------------------------------------------
void usb_vendor_send_callback(void)
{
  app_send_buffer_free = true;
}

//-----------------------------------------------------------------------------
static void send_buffer(void)
{
  app_send_buffer_free = false;
  app_send_zlp = (USB_BUFFER_SIZE == app_send_buffer_ptr);

  // usb_cdc_send(app_send_buffer, app_send_buffer_ptr);

  app_send_buffer_ptr = 0;
}

//-----------------------------------------------------------------------------
void usb_vendor_recv_callback(int size)
{
  app_recv_buffer_ptr = 0;
  app_recv_buffer_size = size;
}

//-----------------------------------------------------------------------------
void usb_configuration_callback(int config)
{

  // usb_cdc_recv(app_recv_buffer, sizeof(app_recv_buffer));
  (void)config;
}




//-----------------------------------------------------------------------------
static void tx_task(void)
{
}

//-----------------------------------------------------------------------------
static void rx_task(void)
{
}

static void gpio_init(void)
{
  HAL_GPIO_BLUE_out();
  HAL_GPIO_RED_out();
  HAL_GPIO_GREEN_out();

  HAL_GPIO_BUTTON_in();
  HAL_GPIO_BUTTON_pullup();
  HAL_GPIO_FLOAT1_in();
  HAL_GPIO_FLOAT1_pullup();
  HAL_GPIO_FLOAT2_in();
  HAL_GPIO_FLOAT2_pullup();

  HAL_GPIO_PUMP1_out();
  HAL_GPIO_PUMP2_out();

  HAL_GPIO_PWRIN_in();

}

//-----------------------------------------------------------------------------
int main(void)
{
  sys_init();
  sys_time_init();
  gpio_init();
  usb_init();
  usb_vendor_init();

  while (1)
  {
    sys_time_task();
    usb_task();
    tx_task();
    rx_task();
    status_task();
  }

  return 0;
}
