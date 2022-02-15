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
#include "commands.h"

/*- Bootloader entry -*/

#define DBL_TAP_MAGIC 0xf02669ef
static volatile uint32_t __attribute__((section(".vectors_ram"))) double_tap;

/*- Definitions -------------------------------------------------------------*/
#define USB_BUFFER_SIZE 64

HAL_GPIO_PIN(RED, A, 17);
HAL_GPIO_PIN(GREEN, A, 22);
HAL_GPIO_PIN(BLUE, A, 23);

HAL_GPIO_PIN(PUMP1, A, 7);
HAL_GPIO_PIN(PUMP2, A, 6);

HAL_GPIO_PIN(FLOAT1, A, 8);
HAL_GPIO_PIN(FLOAT2, A, 9);

HAL_GPIO_PIN(BUTTON, A, 16);

HAL_GPIO_PIN(PWRIN, A, 2);

HAL_GPIO_PIN(USB_DM, A, 24);
HAL_GPIO_PIN(USB_DP, A, 25);

// 1ms system core clock since start
static uint64_t app_system_time = 0;
static bool ignore_wdt_reset = false;
// Counter to flicker the blue LED off on a command received
static uint64_t led_flicker_usb = 0;
// Flag if the USB is mounted on the host
static bool usb_mounted = false;
// Flag that green LED control is set for button control
static bool led_green_button_control = false;

typedef struct
{
  // Timestamp of the pump turnoff point
  uint64_t pump_run_till;
  // Speed of the pump set
  uint8_t pump_speed;
} pump_status_t;

// Pump status register
static pump_status_t pumps[2];

uint8_t usb_serial_number[12];

// Invoked when a control transfer occurred on an interface of this class
// Driver response accordingly to the request and the transfer stage (setup/data/ack)
// return false to stall control endpoint (e.g unsupported request)
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request)
{
  return true;
}

// Invoked when device is mounted
void tud_mount_cb(void)
{
  usb_mounted = true;
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
  usb_mounted = false;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void)remote_wakeup_en;
  usb_mounted = false;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
}

static void wdt_task(void)
{
  if (!ignore_wdt_reset)
    WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY_Val;
}

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

  WDT->CONFIG.reg = WDT_CONFIG_PER_16K | WDT_CONFIG_WINDOW_16K;
  WDT->CTRL.reg = WDT_CTRL_ENABLE;

  SYSCTRL->BOD33.reg = SYSCTRL_BOD33_ACTION_RESET | SYSCTRL_BOD33_ENABLE | SYSCTRL_BOD33_LEVEL(39) | SYSCTRL_BOD33_HYST;
}

//-----------------------------------------------------------------------------
static void sys_time_init(void)
{
  SysTick->LOAD = F_CPU / 1000ul;
  SysTick->VAL = 0;
  SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk;
  NVIC_EnableIRQ(SysTick_IRQn);
  app_system_time = 0;
}

//-----------------------------------------------------------------------------
static void sys_time_task(void)
{
}

//-----------------------------------------------------------------------------
static void status_task(void)
{
  int pwr = HAL_GPIO_PWRIN_read();
  if (!pwr)
  {
    HAL_GPIO_RED_set();
  }
  else
  {
    HAL_GPIO_RED_clr();
  }
  if (usb_mounted)
  {
    if (!led_green_button_control)
    {
      HAL_GPIO_GREEN_clr();
    }
    if (led_flicker_usb < app_system_time)
    {
      HAL_GPIO_BLUE_set();
    }
    else
    {
      HAL_GPIO_BLUE_clr();
    }
  }
  else
  {
    if (!led_green_button_control)
      HAL_GPIO_GREEN_set();
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

// Helper function to set the GPIO decoded
static void _pump_pin_set(uint8_t pump, uint8_t set)
{
  set = set << 1;
  if (set > 200)
    set = 200;
  if (set && set < 40)
    set = 40;
  switch (pump)
  {
  case 0:
    TCC0->CC[2].bit.CC = set;
    break;
  case 1:
    TCC0->CC[3].bit.CC = set;
  default:
    break;
  }
}

static void pump_task(void)
{
  for (int i = 0; i < 2; i++)
  {
    // If there is no new command, turn off the pump
    if (pumps[i].pump_run_till < app_system_time)
    {
      pumps[i].pump_speed = 0;
    }
    _pump_pin_set(i, pumps[i].pump_speed);
  }
}

static void pump_turnon_for(int pump, uint8_t speed, uint64_t millis)
{
  pumps[pump].pump_speed = speed;
  pumps[pump].pump_run_till = app_system_time + millis;
}

static void button_task(void)
{
  static int button_state = 0;
  static int last_button_state = 0;
  static uint64_t last_button_time = 0;
  static int button_counts = 0;

  // Debounce everything
  if (last_button_time + 100 > app_system_time)
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
      led_green_button_control = true;
    else
      led_green_button_control = false;
    if (button_state && button_counts > 10)
      HAL_GPIO_GREEN_set();

    if (button_state && button_counts > 20)
      HAL_GPIO_GREEN_clr();
  }
  else
  {
    // Button state change, do a thing
    if (last_button_state && button_counts > 20)
    {
      pump_turnon_for(1, 100, 5000);
    }
    else if (last_button_state && button_counts > 10)
    {
      pump_turnon_for(0, 100, 5000);
    }
    led_green_button_control = false;
    HAL_GPIO_GREEN_clr();
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

void send_response(void)
{
  int floats = 0;
  if (HAL_GPIO_FLOAT1_read())
  {
    floats = 1;
  }
  if (HAL_GPIO_FLOAT2_read())
  {
    floats |= (1 << 1);
  }
  response_t response = {
      .header = {.response_byte = RESPONSE_STATUS},
      .power_status = !HAL_GPIO_PWRIN_read(),
      .pump1_status = pumps[0].pump_speed,
      .pump2_status = pumps[1].pump_speed,
      .float_status = floats,
      .analog_float_status = 0};
  // echo to web serial
  if (tud_vendor_mounted())
  {
    tud_vendor_write((const void *)&response, sizeof(response));
  }
}

void command_processor_task(void)
{

  if (tud_vendor_available() && tud_vendor_mounted())
  {
    uint8_t buf[64];
    uint32_t count = tud_vendor_read(buf, sizeof(buf));

    led_flicker_usb = app_system_time + 20;
    const command_header_t *header = (const command_header_t *)&buf;
    switch (header->command_byte)
    {
    case COMMAND_SET_OUTPUTS:
    {
      const command_set_outputs_t *command = (const command_set_outputs_t *)&buf;
      pump_turnon_for(0, command->pump1, 5000);
      pump_turnon_for(1, command->pump2, 5000);
    }
    break;
    case COMMAND_READ:
      break;
    case COMMAND_BOOTLOADER_ENTRY:
      double_tap = DBL_TAP_MAGIC;
      // Disable pumps
      pump_turnon_for(0, 0, 5000);
      pump_turnon_for(1, 0, 5000);
      // Schedule the watchdog to jettison us off a cliff
      wdt_task();              // First do a reset
      ignore_wdt_reset = true; // Then ignore the resets
      break;
    default:
      break;
    }
    send_response();
  }
}
void pwm_init(void)
{
  HAL_GPIO_PUMP1_pmuxen(PORT_PMUX_PMUXE_F_Val);
  HAL_GPIO_PUMP2_pmuxen(PORT_PMUX_PMUXE_F_Val);

  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(2) | GCLK_GENCTRL_SRC(GCLK_SOURCE_DFLL48M) |
                      GCLK_GENCTRL_RUNSTDBY | GCLK_GENCTRL_GENEN;
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY)
    ;
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_ID(TCC0_GCLK_ID) | GCLK_CLKCTRL_GEN(2);
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY)
    ;
  PM->APBCMASK.bit.TCC0_ = 1;
  TCC0->CTRLA.bit.SWRST = 1;
  while (TCC0->SYNCBUSY.bit.SWRST)
    ;

  TCC0->WAVE.bit.WAVEGEN = 0x2; // Normal PWM
  TCC0->PER.bit.PER = 200;
  TCC0->CTRLA.bit.PRESCALER = 4;

  TCC0->CC[2].bit.CC = 0x0;
  TCC0->CC[3].bit.CC = 0x0;
  TCC0->CTRLA.bit.ENABLE = 1;
}

//-----------------------------------------------------------------------------
int main(void)
{
  sys_init();
  sys_time_init();
  gpio_init();
  usb_init();
  tusb_init();
  pwm_init();

  while (1)
  {
    sys_time_task();
    status_task();
    button_task();
    tud_task(); // device task
    command_processor_task();
    pump_task();
    wdt_task();
  }

  return 0;
}

/////////////////////////////////////////////
// Interrupt Handlers

void USB_Handler(void)
{
  tud_int_handler(0);
}

void SysTick_Handler(void)
{
  app_system_time++;
}

void SYSCTRL_Handler(void)
{
}
