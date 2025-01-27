#pragma once

#include "Arduino.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define BambuBus_uart USART2
#define BambuBus_pin_tx PA2
#define BambuBus_pin_rx PA3
#define BambuBus_pin_de PA4

    extern void BambuBus_init(bool if_lowspeed_debug);

    /*struct
    {
        int _index = 0;
        uint8_t bus_data[100];
        int length = 100;
        int have_data = 0;
    } BambuBus_statu;*/

    extern void BambuBus_run();


#define BambuBus_use_forwarding_Serial
#ifdef BambuBus_use_forwarding_Serial
#define forwarding_Serial Serial4
    extern void BambuBus_run_forward();
#endif
#ifdef __cplusplus
}
#endif