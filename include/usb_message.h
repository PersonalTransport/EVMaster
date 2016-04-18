/* USBMessage.h
 *
 * Written by Richard Barella Jr. and Joseph O'Banion for the WSU-V CS-Capstone project.
 * 
 * Used as the struct for communication between the LIN network and
 * Android device for the PersonalTransportation project.
 */

#ifndef USBMESSAGE_H
#define USBMESSAGE_H

#define SEND_USB_MESSAGE_LENGTH(packet) (sizeof(struct usb_message_header) + packet->header->length)
#define SEND_USB_MESSAGE(device_handle,packet) AndroidAppRead(device_handle,packet,SEND_USB_MESSAGE_LENGTH(packet))

#include <stdint.h>

enum usb_message_com {
    USBMESSAGE_COMM_SET_VAR   = 0x66,
    USBMESSAGE_COMM_GET_VAR   = 0x77,
    USBMESSAGE_COMM_WARN_VAR  = 0x11
};

struct __attribute__((packed)) usb_message_header {
    uint8_t comm;      // Defines the type of instruction (set, get, warn)
    uint32_t sid;      // Defines what the command is about (lights, blinkers, battery, speed, etc.)
    uint8_t length;    // Defines the length of the data
};

struct __attribute__((packed)) usb_message {
    struct usb_message_header header;
    uint8_t data[8];
};

#endif //USBMESSAGE_H
