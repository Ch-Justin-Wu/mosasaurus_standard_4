#include "stdint.h"




float power_scale(uint8_t remaning_buffer,uint8_t buffer)
{
    float scale;
    float b=10;
    scale = (float)(remaning_buffer - b) / (float)buffer;
    if(remaning_buffer==buffer)
    {
        scale = 1.0f;
    }
    if (scale < 0)
        scale = 0.0f;
    return scale;
}

