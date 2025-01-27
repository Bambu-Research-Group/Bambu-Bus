#pragma once

#include "main.h"

//#define USB_debug_on
#define USB_debug_serial SerialUSB
#define USB_debug_baudrate 115200
#define USB_debug_format SERIAL_8E1

#ifdef __cplusplus
extern "C"
{
#endif
extern void USB_debug_init();
extern void USB_debug_run();
extern uint64_t USB_debug_count64();
extern void USB_debug_time();
extern void USB_debug_write(const void *data);
extern void USB_debug_write_num(const void *data,int num);
#define DEBUG_time_log();
#ifdef USB_debug_on

#define DEBUG(logs) USB_debug_write(logs)
#define DEBUG_num(logs,num) USB_debug_write_num(logs,num)
#define DEBUG_time() USB_debug_time()
#define DEBUG_get_time() USB_debug_count64()
#else
#define DEBUG(logs); ;
#define DEBUG_num(logs,num); ;
#define DEBUG_time(); ;
#endif
#ifdef __cplusplus
}
#endif
