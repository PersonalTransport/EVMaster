// CONFIG4
#pragma config DSWDTPS = DSWDTPSF // DSWDT Postscale Select (1:2,147,483,648 (25.7 days))
#pragma config DSWDTOSC = LPRC // Deep Sleep Watchdog Timer Oscillator Select (DSWDT uses Low Power RC Oscillator (LPRC))
#pragma config RTCOSC = SOSC // RTCC Reference Oscillator Select (RTCC uses Secondary Oscillator (SOSC))
#pragma config DSBOREN = ON // Deep Sleep BOR Enable bit (BOR enabled in Deep Sleep)
#pragma config DSWDTEN = ON // Deep Sleep Watchdog Timer (DSWDT enabled)

// CONFIG3
#pragma config WPFP = WPFP63 // Write Protection Flash Page Segment Boundary (Highest Page (same as page 21))
#pragma config SOSCSEL = IO // Secondary Oscillator Pin Mode Select (SOSC pins have digital I/O functions (RA4, RB4))
#pragma config WUTSEL = LEG // Voltage Regulator Wake-up Time Select (Default regulator start-up time used)
#pragma config WPDIS = WPDIS // Segment Write Protection Disable (Segmented code protection disabled)
#pragma config WPCFG = WPCFGDIS // Write Protect Configuration Page Select (Last page and Flash Configuration words are unprotected)
#pragma config WPEND = WPENDMEM // Segment Write Protection End Page Select (Write Protect from WPFP to the last page of memory)

// CONFIG2
#pragma config POSCMOD = NONE // Primary Oscillator Select (Primary Oscillator disabled)
#pragma config I2C1SEL = PRI // I2C1 Pin Select bit (Use default SCL1/SDA1 pins for I2C1 )
#pragma config IOL1WAY = OFF // IOLOCK One-Way Set Enable (The IOLOCK bit can be set and cleared using the unlock sequence)
#pragma config OSCIOFNC = ON // OSCO Pin Configuration (OSCO pin functions as port I/O (RA3))
#pragma config FCKSM = CSDCMD // Clock Switching and Fail-Safe Clock Monitor (Sw Disabled, Mon Disabled)
#pragma config FNOSC = FRCPLL // Initial Oscillator Select (Fast RC Oscillator with Postscaler and PLL module (FRCPLL))
#pragma config PLL96MHZ = ON // 96MHz PLL Startup Select (96 MHz PLL Startup is enabled automatically on start-up)
#pragma config PLLDIV = NODIV // USB 96 MHz PLL Prescaler Select (Oscillator input used directly (4 MHz input))
#pragma config IESO = OFF // Internal External Switchover (IESO mode (Two-Speed Start-up) disabled)

// CONFIG1
#pragma config WDTPS = PS32768 // Watchdog Timer Postscaler (1:32,768)
#pragma config FWPSA = PR128 // WDT Prescaler (Prescaler ratio of 1:128)
#pragma config WINDIS = OFF // Windowed WDT (Standard Watchdog Timer enabled,(Windowed-mode is disabled))
#pragma config FWDTEN = OFF // Watchdog Timer (Watchdog Timer is disabled)
#pragma config ICS = PGx1 // Emulator Pin Placement Select bits (Emulator functions are shared with PGEC1/PGED1)
#pragma config GWRP = OFF // General Segment Write Protect (Writes to program memory are allowed)
#pragma config GCP = OFF // General Segment Code Protect (Code protection is disabled)
#pragma config JTAGEN = OFF // JTAG Port Enable (JTAG port is disabled)

#include <ev_master.h>
#include <libpic30.h>
#include <stdio.h>
#include <xc.h>

#define RS LATBbits.LATB9
#define EN LATBbits.LATB8
#define D4 LATAbits.LATA0
#define D5 LATAbits.LATA1
#define D6 LATAbits.LATA2
#define D7 LATAbits.LATA3
#include <lcd.h>

#include <usb.h>
#include <usb_host_android.h>
#include <usb_message.h>

#define HEAD_LIGHT_STATE_SID 999653166
#define SIGNAL_LIGHT_STATE_SID 2308980954
#define AXLE_RPM_SID 3524390749
#define BATTERY_VOLTAGE_SID 4052165617
#define USAGE_CURRENT_SID 1512302620
#define CHARGING_CURRENT_SID 3484793322

#define USB_BUFFER_SIZE 64
#define MAX_ALLOWED_CURRENT 500 // mA

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

static bool device_attached = false;
static void* device_handle = NULL;

l_bool read_in_progress = false;
uint32_t read_size = 0;
l_u8 read_buffer[USB_BUFFER_SIZE];

l_bool write_in_progress = false;
uint32_t write_size = 0;
l_u8 write_buffer[USB_BUFFER_SIZE];

enum l_schedule_handle current_schedule = configuration_schedule;

static void master_task_5ms()
{
    l_u8 next_entry = l_sch_tick_UART1();
    if (next_entry == 1) {
        switch (current_schedule) {
        case configuration_schedule: {
            current_schedule = normal_schedule;
            l_sch_set_UART1(current_schedule, 0);
            break;
        }
        default: {
            break;
        }
        }
    }
}

static char str[64];
int main()
{
    {
        unsigned int pll_startup_counter = 600;
        CLKDIVbits.PLLEN = 1;
        while (pll_startup_counter--) {
        }
    }

    // Initialize the LIN interface
    if (l_sys_init())
        return -1;

    // Initialize the interface
    if (l_ifc_init_UART1())
        return -1;

    // Set UART TX to interrupt level 4
    // Set UART RX to interrupt level 4
    struct l_irqmask irqmask = { 4, 4 };
    l_sys_irq_restore(irqmask);

    // Setup the AOA and USB
    AndroidAppStart(&device_info);
    USBHostInit(0);

    TRISBbits.TRISB8 = 0;
    TRISBbits.TRISB9 = 0;
    TRISAbits.TRISA0 = 0; // D4
    TRISAbits.TRISA1 = 0; // D5
    TRISAbits.TRISA2 = 0; // D6
    TRISAbits.TRISA3 = 0; // D7

    Lcd_Init();

    // Setup a 5ms timer
    T1CONbits.TON = 1;
    T1CONbits.TSIDL = 0;
    T1CONbits.TGATE = 0;
    T1CONbits.TCKPS = 1;
    T1CONbits.TSYNC = 0;
    T1CONbits.TCS = 0;
    PR1 = FCY / 8ul / 200ul;

    // Set timer interrupt level to 5
    IEC0bits.T1IE = 1;
    IPC0bits.T1IP = 5;

    current_schedule = configuration_schedule;
    l_sch_set_UART1(current_schedule, 0);

    uint8_t error_code;

    while (true) {
        USBHostTasks();
        AndroidTasks();

        if (!device_attached)
            continue;

        if (read_in_progress) {
            if (AndroidAppIsReadComplete(device_handle, &error_code, &read_size)) {
                read_in_progress = false;
            }
        } else {
            uint32_t index = 0;
            while (index < read_size) {
                struct usb_message* message = (struct usb_message*)(read_buffer + index);
                index += sizeof(struct usb_message_header) + message->header.length;

                Lcd_Clear();
                sprintf(str, "%lx", message->header.sid);
                Lcd_Set_Cursor(1, 1);
                Lcd_Write_String(str);

                sprintf(str, "%x", *((l_u8*)message->data));
                Lcd_Set_Cursor(2, 1);
                Lcd_Write_String(str);

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

        if (!device_attached)
            continue;

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
                if (l_flg_tst_signal_light_state()) {
                    l_flg_clr_signal_light_state();
                    message->header.sid = SIGNAL_LIGHT_STATE_SID;
                    message->header.length = 1;
                    *((l_u8*)message->data) = l_u8_rd_signal_light_state();
                } else if (l_flg_tst_head_light_state()) {
                    l_flg_clr_head_light_state();
                    message->header.sid = HEAD_LIGHT_STATE_SID;
                    message->header.length = 1;
                    *((l_u8*)message->data) = l_u8_rd_head_light_state();
                } else if (l_flg_tst_axle_rpm()) {
                    l_flg_clr_axle_rpm();
                    message->header.sid = AXLE_RPM_SID;
                    message->header.length = 2;
                    *((l_u16*)message->data) = l_u16_rd_axle_rpm();
                } else if (l_flg_tst_battery_voltage()) {
                    l_flg_clr_battery_voltage();
                    message->header.sid = BATTERY_VOLTAGE_SID;
                    message->header.length = 2;
                    *((l_u16*)message->data) = l_u16_rd_battery_voltage();
                } else if (l_flg_tst_usage_current()) {
                    l_flg_clr_usage_current();
                    message->header.sid = USAGE_CURRENT_SID;
                    message->header.length = 2;
                    *((l_u16*)message->data) = l_u16_rd_usage_current();
                } else if (l_flg_tst_charging_current()) {
                    l_flg_clr_charging_current();
                    message->header.sid = CHARGING_CURRENT_SID;
                    message->header.length = 2;
                    *((l_u16*)message->data) = l_u16_rd_charging_current();
                } else {
                    break;
                }
            }

            if (device_attached && write_size > 0) {
                write_in_progress = (AndroidAppWrite(device_handle, write_buffer, write_size) == USB_SUCCESS);
            }
        }
    }
    return -1;
}

struct l_irqmask l_sys_irq_disable()
{
    IEC0bits.U1RXIE = 0;
    IEC0bits.U1TXIE = 0;
    IFS0bits.U1TXIF = 0;
    IFS0bits.U1RXIF = 0;
    struct l_irqmask mask = { IPC2bits.U1RXIP, IPC3bits.U1TXIP };
    return mask;
}

void l_sys_irq_restore(struct l_irqmask previous)
{
    IPC2bits.U1RXIP = previous.rx_level;
    IFS0bits.U1TXIF = 0;
    IEC0bits.U1RXIE = 1;

    IPC3bits.U1TXIP = previous.tx_level;
    IFS0bits.U1RXIF = 0;
    IEC0bits.U1TXIE = 1;
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

void __attribute__((interrupt, no_auto_psv)) _T1Interrupt()
{
    if (IFS0bits.T1IF) {
        IFS0bits.T1IF = 0;
        master_task_5ms();
    }
}

void __attribute__((interrupt, no_auto_psv)) _U1TXInterrupt()
{
    if (IFS0bits.U1TXIF) {
        IFS0bits.U1TXIF = 0;
        l_ifc_tx_UART1();
    }
}

void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt()
{
    if (IFS0bits.U1RXIF) {
        IFS0bits.U1RXIF = 0;
        l_ifc_rx_UART1();
    }
}

void __attribute__((interrupt, auto_psv)) _USB1Interrupt()
{
    USB_HostInterruptHandler();
}
