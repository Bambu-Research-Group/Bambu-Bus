#pragma once


#include "main.h"
#include "BambuBus.h"
#ifdef __cplusplus
extern "C"
{
#endif

#define AMCU_uart uart1
#define AMCU_uart_IRQ UART1_IRQ
#define AMCU_pin_tx 4
#define AMCU_pin_rx 5

#define AMCU_AS5600_SDA 2
#define AMCU_AS5600_SCL 3

    extern void AMCU_init();
    extern void AMCU_run();
#ifdef __cplusplus
}
#endif