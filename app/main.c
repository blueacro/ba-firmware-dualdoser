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
#include "tusb.h"

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

HAL_GPIO_PIN(USB_DM, A, 24);
HAL_GPIO_PIN(USB_DP, A, 25);

static bool app_send_buffer_free = true;
static int app_system_time = 0;
static int app_status = 0;
static int app_last_interval = 0;
static int app_status_timeout = 0;
static bool web_serial_connected = true;

static uint8_t usb_serial_number[9];

// array of pointer to string descriptors
char const *string_desc_arr[] =
    {
        (const char[]){0x09, 0x04}, // 0: is supported language is English (0x0409)
        "StackFoundry LLC",         // 1: Manufacturer
        "ReefVolt DualDoser",       // 2: Product
        "123456",                   // 3: Serials, should use chip ID
        "VendorIF"                  // 4: Vendor Interface
};

static uint16_t _desc_str[32];

// Invoked when received GET STRING DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
  (void)langid;

  uint8_t chr_count;

  if (index == 0)
  {
    memcpy(&_desc_str[1], string_desc_arr[0], 2);
    chr_count = 1;
  }
  else
  {
    // Note: the 0xEE index string is a Microsoft OS 1.0 Descriptors.
    // https://docs.microsoft.com/en-us/windows-hardware/drivers/usbcon/microsoft-defined-usb-descriptors

    if (!(index < sizeof(string_desc_arr) / sizeof(string_desc_arr[0])))
      return NULL;

    const char *str = string_desc_arr[index];
    if (index == 3)
      str = usb_serial_number;
    // Cap at max char
    chr_count = strlen(str);
    if (chr_count > 31)
      chr_count = 31;

    // Convert ASCII string into UTF-16
    for (uint8_t i = 0; i < chr_count; i++)
    {
      _desc_str[1 + i] = str[i];
    }
  }

  // first byte is length (including header), second byte is string type
  _desc_str[0] = (TUSB_DESC_STRING << 8) | (2 * chr_count + 2);

  return _desc_str;
}

// Invoked when a control transfer occurred on an interface of this class
// Driver response accordingly to the request and the transfer stage (setup/data/ack)
// return false to stall control endpoint (e.g unsupported request)
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const * request)
{
  return true;
}


// Invoked when device is mounted
void tud_mount_cb(void)
{
  HAL_GPIO_BLUE_set();
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
  HAL_GPIO_BLUE_clr();
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void) remote_wakeup_en;
  
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
 
}
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
  if (app_system_time % 100 == 0)
  {
    if (app_system_time / 100 != app_last_interval)
    {
      //HAL_GPIO_BLUE_toggle();
      app_last_interval = app_system_time / 100;
    }
  }

  int pwr = HAL_GPIO_PWRIN_read();
  if (!pwr)
  {
    HAL_GPIO_RED_set();
  }
  else
  {
    HAL_GPIO_RED_clr();
  }
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

static void button_handler(void)
{
  static int button_state = 0;
  static int last_button_state = 0;
  static int last_button_time = 0;
  static int button_counts = 0;

  // Debounce everything
  if (last_button_time + 50 > app_system_time)
  {
    return;
  }
  last_button_time = app_system_time;
  last_button_state = button_state;
  button_state = !HAL_GPIO_BUTTON_read();

  if (last_button_state == button_state)
  {
    button_counts += 1;
    if (button_state)
    {
      HAL_GPIO_PUMP1_set();
    }
    else
    {
      HAL_GPIO_PUMP1_clr();
    }
  }
  else
  {
    button_counts = 0;
  }
}

void usb_init(void)
{
  HAL_GPIO_USB_DM_pmuxen(PORT_PMUX_PMUXE_G_Val);
  HAL_GPIO_USB_DP_pmuxen(PORT_PMUX_PMUXE_G_Val);

  PM->APBBMASK.reg |= PM_APBBMASK_USB;

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_ID(USB_GCLK_ID) |
                      GCLK_CLKCTRL_GEN(0);

  USB->DEVICE.CTRLA.bit.SWRST = 1;
  while (USB->DEVICE.SYNCBUSY.bit.SWRST)
    ;

  USB->DEVICE.PADCAL.bit.TRANSN = NVM_READ_CAL(NVM_USB_TRANSN);
  USB->DEVICE.PADCAL.bit.TRANSP = NVM_READ_CAL(NVM_USB_TRANSP);
  USB->DEVICE.PADCAL.bit.TRIM = NVM_READ_CAL(NVM_USB_TRIM);
}
void webserial_task(void)
{
  if ( web_serial_connected )
  {
    if ( tud_vendor_available() && tud_vendor_mounted())
    {
      uint8_t buf[64];
      uint32_t count = tud_vendor_read(buf, sizeof(buf));

      // echo back to both web serial and cdc
      echo_all(buf, count);
    }
  }
}

// send characters to both CDC and WebUSB
void echo_all(uint8_t buf[], uint32_t count)
{
  // echo to web serial
  if ( web_serial_connected && tud_vendor_mounted() )
  {
    tud_vendor_write(buf, count);
  }

}
//-----------------------------------------------------------------------------
int main(void)
{
  sys_init();
  sys_time_init();
  gpio_init();
  usb_init();
  tusb_init();

  while (1)
  {
    sys_time_task();
    status_task();
    button_handler();
    tud_task(); // device task
    webserial_task();
  }

  return 0;
}

void USB_Handler(void)
{
  tud_int_handler(0);
}
