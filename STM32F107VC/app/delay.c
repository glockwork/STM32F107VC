#include "delay.h"
#include "misc.h"

static uint16_t nus;
static uint16_t nms;

void delay_init(void)
{
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
    nus = SystemCoreClock/8000000;
    nms = SystemCoreClock/8000;
}

void delay_us(uint16_t us)
{
    uint32_t temp = 0;
    
    SysTick->LOAD = us * nus;
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_ENABLE_Msk;
    
    do{
		temp = SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));
    
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
    SysTick->VAL   = 0;
}    

void delay_ms(uint16_t ms)
{
    uint32_t temp = 0;
    
    SysTick->LOAD = ms * nms;
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_ENABLE_Msk;
    
    do{
		temp = SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));
    
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
    SysTick->VAL   = 0;
} 
