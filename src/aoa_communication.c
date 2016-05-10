#include <aoa_communication.h>
#include <ev_master.h>

static char manufacturer[] = "Personal Transportation Solutions";
static char model[] = "EvMaster";
static char description[] = "LIN master node";
static char version[] = "0.1";
static char uri[] = "http://ptransportation.com/";
static char serial[] = "N/A";

ANDROID_ACCESSORY_INFORMATION device_info = {
    manufacturer,
    sizeof(manufacturer),
    model,
    sizeof(model),
    description,
    sizeof(description),
    version,
    sizeof(version),
    uri,
    sizeof(uri),
    serial,
    sizeof(serial)
};

static uint8_t error_code;
static bool device_attached = false;
static void* device_handle = NULL;

static l_bool read_in_progress = false;
static uint32_t read_size = 0;
static l_u8 read_buffer[USB_BUFFER_SIZE];

static l_bool write_in_progress = false;
static uint32_t write_size = 0;
static l_u8 write_buffer[USB_BUFFER_SIZE];

void aoa_communication_initialize()
{
    {
        unsigned int pll_startup_counter = 600;
        CLKDIVbits.PLLEN = 1;
        while (pll_startup_counter--) {
        }
    }

    // Setup the AOA and USB
    AndroidAppStart(&device_info);
    USBHostInit(0);
}

static inline void aoa_communication_read()
{
    if (read_in_progress) {
        if (AndroidAppIsReadComplete(device_handle, &error_code, &read_size)) {
            read_in_progress = false;
        }
    } else {
        uint32_t index = 0;
        while (index < read_size) {
            struct usb_message* message = (struct usb_message*)(read_buffer + index);
            index += sizeof(struct usb_message_header) + message->header.length;

            // TODO check command
            switch (message->header.sid) {
            case HEAD_LIGHT_STATE_SID: {
                l_u8_wr_head_light_state(*((l_u8*)message->data));
                break;
            }
            case SIGNAL_LIGHT_STATE_SID: {
                l_u8_wr_signal_light_state(*((l_u8*)message->data));
                break;
            }
            }
        }

        if (device_attached) {
            read_in_progress = (AndroidAppRead(device_handle, read_buffer, USB_BUFFER_SIZE) == USB_SUCCESS);
        }
    }
}

static inline void aoa_communication_write()
{
    if (write_in_progress) {
        uint32_t size;
        if (AndroidAppIsWriteComplete(device_handle, &error_code, &size)) {
            if (size != write_size) {
                // TODO error??
            }
            write_size = 0;
            write_in_progress = false;
        }
    } else {
        while (write_size < (USB_BUFFER_SIZE - MAX_USB_MESSAGE_SIZE)) {
            struct usb_message* message = (struct usb_message*)(write_buffer + write_size);
            message->header.comm = USBMESSAGE_COMM_SET_VAR;
            if (l_flg_tst_motor_controller_duty_cycle()) {
                l_flg_clr_motor_controller_duty_cycle();
                message->header.sid = MOTOR_CONTROLLER_DUTY_CYCLE_SID;
                message->header.length = 2;
                *((l_u16*)message->data) = l_u16_rd_motor_controller_duty_cycle();
                write_size += sizeof(struct usb_message_header) + message->header.length;
            } else if (l_flg_tst_motor_controller_igbt_temperature()) {
                l_flg_clr_motor_controller_igbt_temperature();
                message->header.sid = MOTOR_CONTROLLER_IGBT_TEMPERATURE_SID;
                message->header.length = 2;
                *((l_u16*)message->data) = l_u16_rd_motor_controller_igbt_temperature();
                write_size += sizeof(struct usb_message_header) + message->header.length;
            } else if (l_flg_tst_usage_current()) {
                l_flg_clr_usage_current();
                message->header.sid = USAGE_CURRENT_SID;
                message->header.length = 2;
                *((l_u16*)message->data) = l_u16_rd_usage_current();
                write_size += sizeof(struct usb_message_header) + message->header.length;
            } else {
                break;
            }
        }

        if (device_attached && write_size > 0) {
            write_in_progress = (AndroidAppWrite(device_handle, write_buffer, write_size) == USB_SUCCESS);
        }
    }
}

void aoa_communication_update()
{
    USBHostTasks();
    AndroidTasks();

    if (device_attached)
        aoa_communication_read();

    if (device_attached)
        aoa_communication_write();
}

bool usb_application_event_handler(uint8_t address, USB_EVENT event, void* data, uint32_t size)
{
    switch ((int)event) {
    case EVENT_VBUS_REQUEST_POWER: {
        if (((USB_VBUS_POWER_EVENT_DATA*)data)->current <= (MAX_ALLOWED_CURRENT / 2))
            return true;
        break;
    }
    case EVENT_VBUS_RELEASE_POWER:
    case EVENT_HUB_ATTACH:
    case EVENT_UNSUPPORTED_DEVICE:
    case EVENT_CANNOT_ENUMERATE:
    case EVENT_CLIENT_INIT_ERROR:
    case EVENT_OUT_OF_MEMORY:
    case EVENT_UNSPECIFIED_ERROR:
    case EVENT_DETACH:
    case EVENT_ANDROID_DETACH: {
        device_attached = false;
        return true;
    }
    case EVENT_ANDROID_ATTACH: {
        device_attached = true;
        device_handle = data;
        return true;
    }
    default: {
        break;
    }
    }
    return false;
}

bool usb_application_data_event_handler(uint8_t address, USB_EVENT event, void* data, uint32_t size)
{
    return false;
}

void __attribute__((interrupt, auto_psv)) _USB1Interrupt()
{
    USB_HostInterruptHandler();
}
