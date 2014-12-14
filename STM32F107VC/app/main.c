/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/main.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32_eval.h"
#include <stdio.h>
#include "board.h"
#include "cc112x.h"
#include "delay.h"
#include "cc1120_sniff_mode.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* GPIOA Periph clock enable */
/*******************************************************************************
* Function Name  : MCO_OutPutLSE
* Description    : MCOÊä³öLSE 32.768KHz
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void MCO_OutPut(void)
{
    GPIO_InitTypeDef        GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

    /* Configure PA8 in MCO output mode */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    RCC_MCOConfig(RCC_MCO_SYSCLK);   
}
/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
    uint8_t wr_temp = 40;
    uint8_t rd_temp = 0;
    
    /* Initialize COM port(USART) available onboard */     
    EVAL_USART_Init();

    delay_init();
    printf("Hello!I am STM32107VC...\r\n");
    
    CC1120_Init();
    GPIO_ResetBits(GPIO_Port_CC1120_RESET, GPIO_Pin_CC1120_RESET);
    delay_ms(1);
    GPIO_SetBits(GPIO_Port_CC1120_RESET, GPIO_Pin_CC1120_RESET);
    rf_PowerUpReset();
    registerConfig();
#ifdef CC1120_DEBUG
    printf("registerConfiging OK...\r\n");   
#endif
    MCO_OutPut();
    /* Infinite loop */
    while (1)
    {
//        cc112xSpiWriteReg(CC112X_PKT_LEN, &wr_temp, 1);
        GPIO_ResetBits(GPIO_Port_CC1120_RESET, GPIO_Pin_CC1120_RESET);
        delay_ms(5);
//        cc112xSpiReadReg(CC112X_PKT_LEN, &rd_temp, 1);
        GPIO_SetBits(GPIO_Port_CC1120_RESET, GPIO_Pin_CC1120_RESET);
        delay_ms(5);
//        printf("rd_temp = %d...\r\n",rd_temp);
    }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
