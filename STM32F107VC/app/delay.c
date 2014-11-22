#include "delay.h"

void Delay10us(uint16_t len)
{
    uint16_t index,temp;
    
    for(index = 0;index < len;index ++)
    {
        temp = 720;
        while(temp--);
    }
}    
