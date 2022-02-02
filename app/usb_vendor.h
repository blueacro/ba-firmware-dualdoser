
#ifndef _USB_VENDOR_H_
#define _USB_VENDOR_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "utils.h"
#include "usb_std.h"

/*- Definitions -------------------------------------------------------------*/


enum
{
  USB_CDC_DEVICE_CLASS  = 2,  // USB Communication Device Class
  USB_CDC_COMM_CLASS    = 2,  // CDC Communication Class Interface
  USB_CDC_DATA_CLASS    = 10, // CDC Data Class Interface
};


/*- Prototypes --------------------------------------------------------------*/
void usb_vendor_init(void);
void usb_vendor_send(uint8_t *data, int size);
void usb_vendor_recv(uint8_t *data, int size);

void usb_vendor_send_callback(void);
void usb_vendor_recv_callback(int size);


#endif // _USB_VENDOR_H_
