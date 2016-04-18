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
#pragma config POSCMOD = HS // Primary Oscillator Select (HS Oscillator mode selected)
#pragma config I2C1SEL = PRI // I2C1 Pin Select bit (Use default SCL1/SDA1 pins for I2C1 )
#pragma config IOL1WAY = OFF // IOLOCK One-Way Set Enable (The IOLOCK bit can be set and cleared using the unlock sequence)
#pragma config OSCIOFNC = ON // OSCO Pin Configuration (OSCO pin functions as port I/O (RA3))
#pragma config FCKSM = CSDCMD // Clock Switching and Fail-Safe Clock Monitor (Sw Disabled, Mon Disabled)
#pragma config FNOSC = PRIPLL // Initial Oscillator Select (Primary Oscillator with PLL module (XTPLL, HSPLL, ECPLL))
#pragma config PLL96MHZ = ON // 96MHz PLL Startup Select (96 MHz PLL Startup is enabled automatically on start-up)
#pragma config PLLDIV = DIV3 // USB 96 MHz PLL Prescaler Select (Oscillator input divided by 3 (12 MHz input))
#pragma config IESO = ON // Internal External Switchover (IESO mode (Two-Speed Start-up) enabled)

// CONFIG1
#pragma config WDTPS = PS32768 // Watchdog Timer Postscaler (1:32,768)
#pragma config FWPSA = PR128 // WDT Prescaler (Prescaler ratio of 1:128)
#pragma config WINDIS = OFF // Windowed WDT (Standard Watchdog Timer enabled,(Windowed-mode is disabled))
#pragma config FWDTEN = OFF // Watchdog Timer (Watchdog Timer is disabled)
#pragma config ICS = PGx1 // Emulator Pin Placement Select bits (Emulator functions are shared with PGEC1/PGED1)
#pragma config GWRP = OFF // General Segment Write Protect (Writes to program memory are allowed)
#pragma config GCP = OFF // General Segment Code Protect (Code protection is disabled)
#pragma config JTAGEN = OFF // JTAG Port Enable (JTAG port is disabled)

#include <xc.h>
#include <stdbool.h>
#include <stdint.h>
#include <ev_master.h>

#include <usb.h>
#include <usb_host_android.h>

#define MAX_ALLOWED_CURRENT 500 //mA

static char manufacturer[] = "Personal Transportation Solutions";
static char model[] = "EvMaster";
static char description[] = "LIN master node";
static char version[] = "0.0.1";
static char uri[] = "N/A";
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

int main()
{
    // Initialize the LIN interface
    if (l_sys_init())
        return -1;

    // Initialize the interface
    if (l_ifc_init_UART1())
        return -1;

    // Set UART TX to interrupt level 6
    // Set UART RX to interrupt level 6
    struct l_irqmask irqmask = { 6, 6 };
    l_sys_irq_restore(irqmask);

    // Setup the AOA and USB
    AndroidAppStart(&device_info);
    USBHostInit(0);

    // Setup a 5ms timer
    T1CONbits.TON = 1;
    T1CONbits.TSIDL = 0;
    T1CONbits.TGATE = 0;
    T1CONbits.TCKPS = 0;
    T1CONbits.TSYNC = 0;
    T1CONbits.TCS = 0;
    PR1 = 20000;

    // Set timer interrupt level to 7
    IEC0bits.T1IE = 1;
    IPC0bits.T1IP = 7;

    current_schedule = configuration_schedule;
    l_sch_set_UART1(current_schedule, 0);

    while (true) {
        USBHostTasks();
        AndroidTasks();
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
        // TODO save the handle
        return true;
    }
    case EVENT_ANDROID_ATTACH: {
        // TODO release the handle
        return true;
        break;
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
