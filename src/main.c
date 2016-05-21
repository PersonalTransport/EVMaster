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
#include <ev_master_aoa.h>
#include <xc.h>

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
    ev_master_aoa_initialize();

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

    //Accelerometer setup.
    TRISBbits.TRISB8 = 1;
    __builtin_write_OSCCONL(OSCCON & ~(1<<6));
    RPINR7bits.IC1R = 8;
    __builtin_write_OSCCONL(OSCCON | (1<<6));
    T2CONbits.T32 = 0;
    T2CONbits.TCKPS = 1;
    T2CONbits.TGATE = 0;
    T2CONbits.TCS = 0;
    PR2 = 0xFFFF;
    T2CONbits.TON = 1;
    while(IC1CON1bits.ICBNE != 0)
        (unsigned int)IC1BUF;
    IC1CON1bits.ICSIDL = 0x00;
    IC1CON1bits.ICTSEL = 0x07;
    IC1CON1bits.ICI    = 0x00;
    IC1CON1bits.ICM    = 0x03;
    IC1CON2bits.IC32 = 0x00;
    IC1CON2bits.ICTRIG = 0x01;
    IC1CON2bits.SYNCSEL = 0b01100;
            
    IEC0bits.IC1IE = 1;
    IPC0bits.IC1IP = 3;
    IC1TMR = 0;
    
    while (true) {
        ev_master_aoa_update();
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

void __attribute__((interrupt, no_auto_psv)) _T1Interrupt()
{
    if (IFS0bits.T1IF) {
        IFS0bits.T1IF = 0;
        master_task_5ms();
    }
}

void __attribute__((interrupt,no_auto_psv)) _IC1Interrupt() {
    if(IFS0bits.IC1IF) {
        IFS0bits.IC1IF = 0;
        if(IC1CON1bits.ICM == 0x03) {
            IC1CON1bits.ICM = 0x02;
            TMR2 = 0;
        } else if(IC1CON1bits.ICM == 0x02) {
            IC1CON1bits.ICM = 0x03;
            l_u16_wr_x_acceleration(TMR2);
        }
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
