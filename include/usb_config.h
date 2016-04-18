#define _USB_CONFIG_VERSION_MAJOR 0
#define _USB_CONFIG_VERSION_MINOR 0
#define _USB_CONFIG_VERSION_DOT 12
#define _USB_CONFIG_VERSION_BUILD 0

#define USB_SUPPORT_HOST

#define USB_PING_PONG_MODE USB_PING_PONG__FULL_PING_PONG

#define NUM_TPL_ENTRIES 7
#define NUM_CLIENT_DRIVER_ENTRIES 2

#define USB_ENABLE_TRANSFER_EVENT

#define USB_HOST_APP_DATA_EVENT_HANDLER usb_application_data_event_handler
#define USB_ENABLE_1MS_EVENT

#define ANDROID_DEVICE_ATTACH_TIMEOUT 3000

#define USB_MAX_GENERIC_DEVICES 1
#define USB_NUM_CONTROL_NAKS 20
#define USB_SUPPORT_INTERRUPT_TRANSFERS
#define USB_SUPPORT_BULK_TRANSFERS
#define USB_NUM_INTERRUPT_NAKS 3
#define USB_INITIAL_VBUS_CURRENT (100 / 2)
#define USB_INSERT_TIME (250 + 1)
#define USB_HOST_APP_EVENT_HANDLER usb_application_event_handler

#define USBTasks()      \
    {                   \
        USBHostTasks(); \
        AndroidTasks(); \
    }

#define USBInitialize(x) \
    {                    \
        USBHostInit(x);  \
    }
