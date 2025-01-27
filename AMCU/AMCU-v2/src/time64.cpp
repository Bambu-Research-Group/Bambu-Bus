#include "time64.h"
uint64_t _time64_time_H=0;
uint32_t _time64_time_L=0;
uint64_t get_time64()
{
    uint32_t T=millis();
    if(T<_time64_time_L)
    {
        _time64_time_H+=0x100000000;
    }
    _time64_time_L=T;
    return _time64_time_H|_time64_time_L;
}