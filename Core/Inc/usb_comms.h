/*
 * usb_comms.h
 *
 *  Created on: May 26, 2025
 *      Author: hilmo
 */
#ifndef INC_USB_COMMS_H_
#define INC_USB_COMMS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "usbd_cdc_if.h"
#include <stdint.h>
#include "string.h"


void send_message(const char *msg);
void read_message(char *dest_buffer, uint16_t max_len);

#ifdef __cplusplus
}
#endif

#endif /* INC_USB_COMMS_H_ */
