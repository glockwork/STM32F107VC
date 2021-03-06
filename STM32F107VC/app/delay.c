#include "delay.h"
#include "misc.h"
#include <stdio.h>

static uint16_t nus;
static uint16_t nms;

void delay_init(void)
{
    RCC_ClocksTypeDef SystemCoreClock;
    
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
    RCC_GetClocksFreq(&SystemCoreClock);
#ifdef Debug
    printf("SystemCoreClock.HCLK_Frequency = %d...\r\n",SystemCoreClock.HCLK_Frequency);
    printf("SystemCoreClock.PCLK1_Frequency = %d...\r\n",SystemCoreClock.PCLK1_Frequency);
    printf("SystemCoreClock.PCLK2_Frequency = %d...\r\n",SystemCoreClock.PCLK2_Frequency);
    printf("SystemCoreClock.SYSCLK_Frequency = %d...\r\n",SystemCoreClock.SYSCLK_Frequency);
#endif
    nus = SystemCoreClock.HCLK_Frequency/8000000;	
	nms = (uint16_t)nus*1000;  
}

void delay_us(uint16_t us)
{
    uint32_t temp = 0;
    
    SysTick->LOAD = us * nus;
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_ENABLE_Msk;
    
    do{
		temp = SysTick->CTRL;
	}while(temp&0x01&&!(temp&(1<<16)));
    
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
	}while(temp&0x01&&!(temp&(1<<16)));
    
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
    SysTick->VAL   = 0;
} 
