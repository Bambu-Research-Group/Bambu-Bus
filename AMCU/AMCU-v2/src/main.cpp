#include "main.h"
#include "pinDefinitions.h"

#include "math.h"
#include "pico.h"
#include "pinDefinitions.h"
#include "mbed.h"

#include "AMCU.h"


void setup()
{
    USB_debug_init();
    AMCU_init();
}

void loop()
{
    while (1)
    {
        USB_debug_run();
        AMCU_run();
    }
}