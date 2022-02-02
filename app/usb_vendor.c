#include <stdbool.h>
#include <stdalign.h>
#include <string.h>
#include "utils.h"
#include "usb.h"
#include "usb_std.h"
#include "usb_vendor.h"
#include "usb_descriptors.h"

/*- Prototypes --------------------------------------------------------------*/
static void usb_vendor_ep_comm_callback(int size);
static void usb_vendor_ep_send_callback(int size);
static void usb_vendor_ep_recv_callback(int size);

static int usb_cdc_serial_state;
static bool usb_cdc_comm_busy;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
void usb_vendor_init(void)
{
  usb_set_callback(USB_CDC_EP_SEND, usb_vendor_ep_send_callback);
  usb_set_callback(USB_CDC_EP_RECV, usb_vendor_ep_recv_callback);
}

static void usb_vendor_ep_send_callback(int size)
{
  usb_vendor_recv_callback(size);
}

static void usb_vendor_ep_recv_callback(int size)
{
  usb_vendor_send_callback();
  (void)size;
}

void usb_vendor_send(uint8_t *data, int size)
{
  return usb_send(USB_CDC_EP_SEND, data, size);
}

void usb_vendor_recv(uint8_t *data, int size)
{
  return usb_recv(USB_CDC_EP_RECV, data, size);
}
//-----------------------------------------------------------------------------
bool usb_class_handle_request(usb_request_t *request)
{
  int length = request->wLength;

  usb_control_send_zlp();
  return true;
}
