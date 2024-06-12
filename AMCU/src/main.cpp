#include <Arduino.h>
#include "BambuBus.h"

#define LED_R PA13
#define LED_G PA14
#define LED_Y PA15
#define LED_B PB4


void setup()
{
    
    pinMode(LED_R, OUTPUT);
    pinMode(LED_G, OUTPUT);
    pinMode(LED_Y, OUTPUT);
    pinMode(LED_B, OUTPUT);

    digitalWrite(LED_R,0);
    digitalWrite(LED_G,0);
    digitalWrite(LED_Y,0);
    digitalWrite(LED_B,0);
    BambuBus_init(false);
    Serial4.begin(1228800,SERIAL_8E1);

}

void loop()
{
    while(1)
    {
        //BambuBus_run_forward();
        BambuBus_run();
        
    }
}
