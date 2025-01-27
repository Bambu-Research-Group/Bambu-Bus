#pragma once

#include "main.h"
#define Bambubus_version 4
#ifdef __cplusplus
extern "C"
{
#endif

#define BambuBus_uart uart0
#define BambuBus_uart_IRQ UART0_IRQ
#define BambuBus_pin_tx 0
#define BambuBus_pin_rx 1
#define BambuBus_pin_de 6
    enum _filament_status
    {
        offline,
        online,
        NFC_waiting
    };
    enum _filament_motion_state_set
    {
        need_pull_back,
        need_send_out,
        on_use,
        idle
    };
    enum package_type
    {
        BambuBus_package_ERROR = -1,
        BambuBus_package_NONE = 0,
        BambuBus_package_filament_motion_short,
        BambuBus_package_filament_motion_long,
        BambuBus_package_online_detect,
        BambuBus_package_REQx6,
        BambuBus_package_NFC_detect,
        BambuBus_package_set_filament,
        BambuBus_long_package_MC_online,
        BambuBus_longe_package_filament,
        BambuBus_long_package_version,
        BambuBus_package_heartbeat,
        BambuBus_package_ETC,
        __BambuBus_package_packge_type_size
    };
    extern void BambuBus_init();
    extern package_type BambuBus_run();


    extern bool Bambubus_flash_read();
    extern void Bambubus_set_need_to_save();
    extern int get_now_filament_num();
    extern void reset_filament_meters(int num);
    extern void add_filament_meters(int num, float meters);
    extern float get_filament_meters(int num);
    extern uint16_t get_filament_pressure(int num);
    extern void set_filament_online(int num, bool if_online);
    _filament_motion_state_set get_filament_motion(int num);
#ifdef __cplusplus
}
#endif