#include "mcc_generated_files/mcc.h"
#include "lin_generated_files/ev_master.h"
#include "lin_generated_files/ev_master_aoa.h"

enum l_schedule_handle current_schedule = configuration_schedule;

void TMR1_CallBack()
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
    SYSTEM_Initialize();
    
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

    current_schedule = configuration_schedule;
    l_sch_set_UART1(current_schedule, 0);
    
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