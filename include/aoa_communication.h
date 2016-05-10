#ifndef AOA_COMMUNICATION_H
#define AOA_COMMUNICATION_H

#include <usb.h>
#include <usb_host_android.h>

#define MOTOR_CONTROLLER_DUTY_CYCLE_SID 1398608173ul
#define MOTOR_CONTROLLER_IGBT_TEMPERATURE_SID 316875851ul
#define HEAD_LIGHT_STATE_SID 999653166ul
#define SIGNAL_LIGHT_STATE_SID 2308980954ul
#define AXLE_RPM_SID 3524390749ul
#define BATTERY_VOLTAGE_SID 4052165617ul
#define USAGE_CURRENT_SID 1512302620ul
#define CHARGING_CURRENT_SID 3484793322ul

#define USB_BUFFER_SIZE 64
#define MAX_ALLOWED_CURRENT 500 // mA

enum usb_message_com {
    USBMESSAGE_COMM_SET_VAR = 0x66,
    USBMESSAGE_COMM_GET_VAR = 0x77,
    USBMESSAGE_COMM_WARN_VAR = 0x11
};

struct __attribute__((packed)) usb_message_header {
    uint8_t comm; // Defines the type of instruction (set, get, warn)
    uint32_t sid; // Defines what the command is about (lights, blinkers, battery, speed, etc.)
    uint8_t length; // Defines the length of the data
};

struct __attribute__((packed)) usb_message {
    struct usb_message_header header;
    uint8_t data[8];
};

#define MAX_USB_MESSAGE_SIZE sizeof(struct usb_message)

void aoa_communication_initialize();

void aoa_communication_update();

#endif //AOA_COMMUNICATION_H
