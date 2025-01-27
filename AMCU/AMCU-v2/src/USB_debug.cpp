#include "USB_debug.h"
#include <inttypes.h>
#include "multicore.h"
#include "mbed.h"

uint8_t _USB_debug_BUF_datas[100];
mbed::Timer USB_debug_timer;



void USB_debug_init()
{
#ifdef USB_debug_on
    USB_debug_timer.start();
    USB_debug_serial.begin(USB_debug_baudrate, USB_debug_format);

#endif
}

uint64_t USB_debug_count64()
{
    return USB_debug_timer.elapsed_time().count();
}

void USB_debug_time()
{
#ifdef USB_debug_on
    unsigned char data[100];
    uint64_t _time64 = USB_debug_timer.elapsed_time().count();
    int i = sprintf((char *)data, "\n[%llu s", _time64 / 1000);
    //
    i = sprintf((char *)data, "%llu ms]", _time64 % 1000);
    //
#endif
}

void USB_debug_write(const void *data)
{
#ifdef USB_debug_on
    int i = strlen((const char*)data);
    USB_debug_serial.write((const char *)data,i);
#endif
}
void USB_debug_write_num(const void *data, int num)
{
#ifdef USB_debug_on
    USB_debug_serial.write((const char *)data,num);
#endif
}

void USB_debug_run()
{

}